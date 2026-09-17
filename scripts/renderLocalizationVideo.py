#!/usr/bin/env python3
"""Render synchronized MnCAV localization and recorded LiDAR perception.

First exportLocalizationVideoData in MATLAB and prepareLocalizationVideoClouds.py. Then:
uv run --offline --with numpy --with pillow python scripts/renderLocalizationVideo.py
Use --preview-only to inspect six frames before encoding the full replay.
USGS/USDA aerial orthoimagery supplies geographic context only.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import time
import urllib.parse
import urllib.request

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
BG = '#09121e'
PANEL = '#111f2e'
EDGE = '#26394b'
WHITE = '#edf4fa'
MUTED = '#94aabd'
TRUTH = '#51e4c2'
EST = '#ffc364'
SEMANTIC = ['#6c8190', '#ff7957', '#59bfff', '#ee84e9']
W, H = 1920, 1080
MAP = (24, 172, 1104, 696)
LIDAR = (1148, 172, 748, 430)
DETAIL = (1148, 618, 330, 250)
METRICS = (1494, 618, 402, 250)
TIMELINE = (24, 884, 1872, 152)


def rgb(color):
    return tuple(bytes.fromhex(color.lstrip('#')))


def font(size, bold=False):
    path = Path.home() / '.local/share/fonts/Inter/Inter.ttc'
    if path.exists():
        return ImageFont.truetype(str(path), size, index=14 if bold else 0)
    path = '/usr/share/fonts/truetype/dejavu/DejaVuSans' + ('-Bold' if bold else '') + '.ttf'
    return ImageFont.truetype(path, size)


FONTS = {(s, b): font(s, b) for s in [13, 14, 15, 16, 17, 18, 20, 22, 24, 27, 30, 32, 34] for b in [False, True]}


def text(draw, xy, value, size=18, color=WHITE, bold=False, anchor=None):
    draw.text(xy, str(value), font=FONTS[size, bold], fill=color, anchor=anchor)


def panel(draw, box, title):
    x, y, w, h = box
    draw.rounded_rectangle((x, y, x+w, y+h), radius=12, fill=PANEL, outline=EDGE, width=1)
    text(draw, (x+18, y+12), title, 18, WHITE, True)


def fetch_imagery(out, trajectory):
    image_file = out / 'orthoimagery.jpg'
    metadata_file = out / 'imagery_metadata.json'
    if image_file.exists() and metadata_file.exists():
        return Image.open(image_file).convert('RGB'), json.loads(metadata_file.read_text())
    bbox = [float(trajectory['truthX'].min()-165), float(trajectory['truthY'].min()-165),
            float(trajectory['truthX'].max()+165), float(trajectory['truthY'].max()+165)]
    width, height = [round((bbox[k+2]-bbox[k])/.31) for k in range(2)]
    assert max(width, height) <= 4000
    service = 'https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPPlus/ImageServer'
    args = {'bbox': ','.join(map(str, bbox)), 'bboxSR': 32615, 'imageSR': 32615,
            'size': f'{width},{height}', 'format': 'jpg', 'bandIds': '0,1,2',
            'interpolation': 'RSP_BilinearInterpolation', 'f': 'json'}
    request = service+'/exportImage?'+urllib.parse.urlencode(args)
    with urllib.request.urlopen(request, timeout=60) as stream:
        meta = json.load(stream)
    if 'error' in meta:
        raise RuntimeError(meta)
    with urllib.request.urlopen(meta['href'], timeout=60) as stream:
        image_file.write_bytes(stream.read())
    with urllib.request.urlopen(service+'?f=pjson', timeout=30) as stream:
        description = json.load(stream)
    meta.update({'requestedUrl': request, 'serviceUrl': service,
                 'retrievedAt': datetime.now(timezone.utc).isoformat(), 'requestedBbox': bbox,
                 'copyrightText': description['copyrightText'], 'description': description['description'],
                 'role': 'Geographic context only; no image-based accuracy claim',
                 'imageryType': 'Aerial orthoimagery, not a live satellite image'})
    metadata_file.write_text(json.dumps(meta, indent=2)+'\n')
    return Image.open(image_file).convert('RGB'), meta


class Renderer:
    def __init__(self, out):
        self.out = out
        self.tr = np.genfromtxt(out/'trajectory.csv', delimiter=',', names=True)
        self.fr = np.genfromtxt(out/'frames.csv', delimiter=',', names=True)
        self.meta = json.loads((out/'data_metadata.json').read_text())
        self.cloud_file = np.load(out/'display_clouds.npz')
        self.clouds = [self.cloud_file[f'frame_{int(k):04d}'] for k in self.fr['frame']]
        self.frame_time = self.fr['time']
        self.t = self.tr['time']
        self.duration = float(self.frame_time[-1])
        self.truth = np.column_stack([self.tr[n] for n in ['truthX', 'truthY', 'truthYaw']])
        self.estimate = np.column_stack([self.tr[n] for n in ['estimateX', 'estimateY', 'estimateYaw']])
        assert len(self.fr) == len(self.clouds) == self.meta['frames']
        assert np.array_equal(self.t,self.frame_time), 'State and perception clocks must match.'
        assert np.all(np.diff(self.t) > 0) and np.all(np.diff(self.frame_time) > 0)
        self.base, self.imagery = fetch_imagery(out, self.tr)
        ex = self.imagery['extent']
        assert ex['spatialReference']['wkid'] == 32615
        self.extent = np.array([ex[n] for n in ['xmin', 'ymin', 'xmax', 'ymax']])
        self.cloud_cache = {}
        self.static = self.make_static()
        self.overview = self.make_overview()
        self.preview_times = [0., 20., 50., 82., 100., self.duration]

    def sample(self, t):
        # Animation repeats real 10 Hz outputs; it does not synthesize states.
        index = max(0,int(np.searchsorted(self.t,t,side='right')-1))
        return self.truth[index],self.estimate[index],self.tr['speedMps'][index],index

    def map_crop(self, extent, size):
        xmin, ymin, xmax, ymax = self.extent
        x0, y0, x1, y1 = extent
        box = ((x0-xmin)/(xmax-xmin)*self.base.width,
               (ymax-y1)/(ymax-ymin)*self.base.height,
               (x1-xmin)/(xmax-xmin)*self.base.width,
               (ymax-y0)/(ymax-ymin)*self.base.height)
        assert min(box[:2]) >= -1 and box[2] <= self.base.width+1 and box[3] <= self.base.height+1
        return self.base.transform(size, Image.Transform.EXTENT, box, Image.Resampling.BICUBIC)

    def make_static(self):
        im = Image.new('RGB', (W, H), BG);d = ImageDraw.Draw(im)
        text(d, (24, 20), 'MnCAV  /  VEHICLE LOCALIZATION', 30, WHITE, True)
        text(d, (25, 58), 'Mississippi River drive  ·  Ground truth: INSPVA  ·  Aligned BESTPOS + LiDAR', 17, MUTED)
        for box, title in [(LIDAR, 'CURRENT LiDAR PERCEPTION'), (DETAIL, 'POSITION OFFSET'),
                           (METRICS, 'LOCALIZATION ACCURACY'), (TIMELINE, 'POSITION ERROR OVER TIME')]:
            panel(d, box, title)
        labels = ['POSITION ERROR', 'HEADING ERROR', 'VEHICLE SPEED', 'LiDAR FRAME', 'LiDAR MATCH']
        for k, label in enumerate(labels):
            x = 24+k*378
            d.rounded_rectangle((x, 92, x+360, 156), radius=10, fill=PANEL)
            text(d, (x+16, 100), label, 13, MUTED, True)
        # Fixed-scale error plot, with only past data added during rendering.
        x, y, w, h = TIMELINE
        for cm in [0, 10, 20, 30, 40]:
            yy = y+122-cm*1.8
            d.line((x+66, yy, x+w-18, yy), fill=EDGE, width=1)
            text(d, (x+54, yy), str(cm), 13, MUTED, anchor='rm')
        for sec in range(0, 121, 20):
            xx = x+66+sec/120*(w-84)
            text(d, (xx, y+128), f'{sec}s', 13, MUTED, anchor='mt')
        text(d, (x+14, y+88), 'cm', 13, MUTED)
        text(d, (x+w-18, y+14), 'Amber bands: no new full-pose LiDAR measurement', 14, MUTED, anchor='rt')
        for k in range(len(self.fr)-1):
            if self.fr['measurementStatus'][k] != 1:
                xa=x+66+self.frame_time[k]/120*(w-84)
                xb=x+66+self.frame_time[k+1]/120*(w-84)
                d.rectangle((xa, y+45, xb, y+122), fill='#322b22')
        # Local offset: true metric scale, referenced to current ground truth.
        x, y, w, h = DETAIL;cx=x+165;cy=y+143;scale=2.0
        for cm in [-40, -20, 0, 20, 40]:
            d.line((cx+cm*scale, cy-80, cx+cm*scale, cy+80), fill=EDGE, width=1)
            d.line((cx-80, cy+cm*scale, cx+80, cy+cm*scale), fill=EDGE, width=1)
        d.line((cx-90,cy,cx+90,cy),fill=MUTED,width=1)
        d.line((cx,cy-90,cx,cy+90),fill=MUTED,width=1)
        text(d,(x+18,y+38),'Relative to current INSPVA position',13,MUTED)
        text(d,(cx+97,cy),'E',14,MUTED,anchor='lm');text(d,(cx+12,cy-81),'N',14,MUTED,anchor='lm')
        text(d,(x+18,y+h-21),'20 cm / grid  ·  last 5 s shown',13,MUTED)
        x,y,w,h=METRICS
        text(d,(x+18,y+48),'Full-run RMSE',17,MUTED)
        text(d,(x+w-18,y+46),f"{100*self.meta['positionRmseM']:.2f} cm",27,EST,True,'rt')
        text(d,(x+18,y+88),'Median error',17,MUTED)
        text(d,(x+w-18,y+88),f"{100*self.meta['positionMedianM']:.2f} cm",22,WHITE,True,'rt')
        text(d,(x+18,y+129),f"Matched-frame RMSE ({self.meta['acceptedLidarSamples']:,} frames)",14,MUTED)
        text(d,(x+18,y+152),f"Fusion {100*self.meta['acceptedFusionPositionRmseM']:.2f} cm  /  LiDAR {100*self.meta['acceptedLidarPositionRmseM']:.2f} cm",16,WHITE)
        text(d,(x+18,y+188),f"{self.meta['localizationSamples']:,} estimates  ·  {self.meta['localizationRateHz']:.0f} Hz",16,WHITE)
        text(d,(x+18,y+216),'Wheel / IMU / lateral observer aiding',14,MUTED)
        text(d,(24,1051),'Imagery: USGS / USDA, The National Map  ·  Aerial orthophoto, geographic context only',14,MUTED)
        text(d,(1896,1051),'Reference-assisted map / matching  ·  true map scale',14,MUTED,anchor='rt')
        return im

    def make_overview(self):
        # Geographic aspect is preserved; this inset is a route locator.
        center=np.mean(np.array([self.truth[:,:2].min(axis=0),self.truth[:,:2].max(axis=0)]),axis=0)
        height=(np.ptp(self.truth[:,1])+80);width=height*176/244
        self.overview_extent=np.r_[center-[width,height]/np.array([2,2]),center+[width,height]/np.array([2,2])]
        im=self.map_crop(self.overview_extent,(176,244));d=ImageDraw.Draw(im)
        p=self.xy_pixels(self.truth[::8,:2],self.overview_extent,(176,244))
        d.line([tuple(v) for v in p],fill='#b2c1ca',width=2)
        return im

    @staticmethod
    def xy_pixels(xy,extent,size):
        xy=np.atleast_2d(xy);x0,y0,x1,y1=extent
        return np.column_stack(((xy[:,0]-x0)/(x1-x0)*size[0],(y1-xy[:,1])/(y1-y0)*size[1]))

    @staticmethod
    def vehicle(d,point,yaw,color,scale,outline=False):
        # Position is the recorded output point; icon geometry is illustrative.
        direction=np.array([np.cos(yaw),-np.sin(yaw)]);left=np.array([-direction[1],direction[0]])
        p=np.asarray(point);front=p+direction*2.9*scale;rear=p-direction*2.3*scale
        poly=[front, p+direction*1.1*scale+left*1.05*scale,rear+left*1.05*scale,
              rear-left*1.05*scale,p+direction*1.1*scale-left*1.05*scale]
        if outline:d.line([tuple(v) for v in poly+[poly[0]]],fill=color,width=3)
        else:d.polygon([tuple(v) for v in poly],fill=color)
        d.ellipse((p[0]-3,p[1]-3,p[0]+3,p[1]+3),fill=WHITE)

    def map_panel(self,t,truth,estimate):
        _,_,w,h=MAP;span=185.;height=span*h/w
        center=truth[:2];extent=np.r_[center-[span/2,height/2],center+[span/2,height/2]]
        im=self.map_crop(extent,(w,h));d=ImageDraw.Draw(im)
        ix=np.flatnonzero(self.t<=t)
        for xy,color,width in [(self.truth[ix,:2],TRUTH,5),(self.estimate[ix,:2],EST,2)]:
            p=self.xy_pixels(xy,extent,(w,h))
            # PIL clips line segments to the panel; no coordinate exaggeration.
            if len(p)>1:d.line([tuple(v) for v in p],fill=color,width=width)
        pp=self.xy_pixels(np.stack([truth[:2],estimate[:2]]),extent,(w,h))
        # A subtle dark halo makes nearly coincident physical positions legible.
        for p in pp:d.ellipse((p[0]-21,p[1]-21,p[0]+21,p[1]+21),outline='#17252d',width=3)
        self.vehicle(d,pp[0],truth[2],TRUTH,w/span)
        self.vehicle(d,pp[1],estimate[2],EST,w/span,True)
        d.rounded_rectangle((14,14,w-14,63),radius=8,fill='#101c29')
        text(d,(29,28),'GEOGRAPHIC VIEW',18,WHITE,True)
        d.line((287,38,323,38),fill=TRUTH,width=5);text(d,(333,27),'INSPVA ground truth',17,WHITE)
        d.line((582,38,618,38),fill=EST,width=3);text(d,(628,27),'Localization estimate',17,WHITE)
        text(d,(w-29,29),'1× speed',16,MUTED,anchor='rt')
        inset=self.overview.copy();di=ImageDraw.Draw(inset)
        p=self.xy_pixels(truth[:2],self.overview_extent,inset.size)[0]
        di.ellipse((p[0]-5,p[1]-5,p[0]+5,p[1]+5),fill=TRUTH,outline=WHITE,width=1)
        d.rounded_rectangle((15,80,207,362),radius=8,fill='#101c29')
        im.paste(inset,(23,109));text(d,(28,86),'ROUTE OVERVIEW',13,WHITE,True)
        # North arrow and a metric scale in the same grid coordinates.
        d.polygon([(w-44,86),(w-52,112),(w-44,107),(w-36,112)],fill=WHITE)
        text(d,(w-44,121),'N',14,WHITE,True,'mt')
        bar=20*w/span
        d.rounded_rectangle((16,h-63,242,h-14),radius=8,fill='#101c29')
        d.line((30,h-34,30+bar,h-34),fill=WHITE,width=3)
        d.line((30,h-39,30,h-29),fill=WHITE,width=2)
        d.line((30+bar,h-39,30+bar,h-29),fill=WHITE,width=2)
        text(d,(30+bar+13,h-44),'20 m',17,WHITE)
        d.rounded_rectangle((w-384,h-63,w-16,h-14),radius=8,fill='#101c29')
        text(d,(w-369,h-48),'North-up  ·  true scale  ·  past trajectory',15,WHITE)
        return im

    @staticmethod
    def project_cloud(points,size):
        eye=np.array([-17.,-23.,22.]);target=np.array([15.,0.,-1.2]);forward=target-eye;forward/=np.linalg.norm(forward)
        right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right);up=np.cross(right,forward)
        p=points-eye;depth=p@forward
        focal=590
        return np.column_stack((size[0]/2+focal*(p@right)/depth,
                                size[1]*.53-focal*(p@up)/depth)),depth

    def cloud_panel(self,index):
        if index in self.cloud_cache:return self.cloud_cache[index]
        _,_,w,h=LIDAR;im=Image.new('RGB',(w,h-44),PANEL);d=ImageDraw.Draw(im)
        # Grid and range marks are drawn in the same vehicle frame as points.
        for xx in range(-10,51,10):
            xy,z=self.project_cloud(np.array([[xx,-25,-1.8],[xx,25,-1.8]]),im.size)
            d.line([tuple(v) for v in xy],fill='#223344',width=1)
        for yy in range(-20,21,10):
            xy,z=self.project_cloud(np.array([[-10,yy,-1.8],[50,yy,-1.8]]),im.size)
            d.line([tuple(v) for v in xy],fill='#223344',width=1)
        cloud=self.clouds[index];xy,depth=self.project_cloud(cloud[:,:3],im.size)
        ok=(depth>1)&(xy[:,0]>=2)&(xy[:,0]<w-2)&(xy[:,1]>=2)&(xy[:,1]<im.height-31)
        arr=np.asarray(im).copy()
        for cls in range(4):
            chosen=ok&(cloud[:,3]==cls);p=np.round(xy[chosen]).astype(int)
            color=rgb(SEMANTIC[cls])
            offsets=[(0,0)] if cls==0 else [(a,b) for a in range(-1,2) for b in range(-1,2)]
            for a,b in offsets:arr[p[:,1]+b,p[:,0]+a]=color
        im=Image.fromarray(arr);d=ImageDraw.Draw(im)
        body=np.array([[-2.3,-1,-1.6],[2.8,-1,-1.6],[2.8,1,-1.6],[-2.3,1,-1.6],[-2.3,-1,-1.6]])
        p,_=self.project_cloud(body,im.size);d.line([tuple(v) for v in p],fill=WHITE,width=2)
        p,_=self.project_cloud(np.array([[0,0,-1.6],[4,0,-1.6],[3,.6,-1.6],[4,0,-1.6],[3,-.6,-1.6]]),im.size)
        d.line([tuple(v) for v in p],fill=WHITE,width=2)
        labels=[('Curb',int(self.fr['curbPoints'][index])),('Pole',int(self.fr['polePoints'][index])),
                ('Traffic sign',int(self.fr['trafficSignPoints'][index]))]
        d.rectangle((0,im.height-30,w,im.height),fill=PANEL)
        for k,(label,count) in enumerate(labels):
            x=18+k*235;d.ellipse((x,im.height-21,x+7,im.height-14),fill=SEMANTIC[k+1])
            text(d,(x+15,im.height-26),f'{label}  {count}',16,WHITE)
        text(d,(w-16,8),'Vehicle frame  ·  10 m grid',13,MUTED,anchor='rt')
        self.cloud_cache={index:im}
        return im

    def render(self,t):
        truth,estimate,speed,index=self.sample(t);delta=estimate[:2]-truth[:2]
        error=100*np.linalg.norm(delta);yaw=np.arctan2(np.sin(estimate[2]-truth[2]),np.cos(estimate[2]-truth[2]))
        im=self.static.copy();im.paste(self.map_panel(t,truth,estimate),MAP[:2])
        im.paste(self.cloud_panel(index),(LIDAR[0],LIDAR[1]+40));d=ImageDraw.Draw(im)
        text(d,(1896,20),f'{int(t//60):02d}:{t%60:04.1f}  /  {int(self.duration//60):02d}:{self.duration%60:04.1f}',30,WHITE,True,'rt')
        age=t-self.frame_time[index]
        text(d,(1896,60),f'Frame age {age*1000:.0f} ms  ·  10 Hz states held for display',15,MUTED,False,'rt')
        status=int(self.fr['measurementStatus'][index])
        values=[f'{error:.1f} cm',f'{abs(np.rad2deg(yaw)):.3f}°',f'{speed*3.6:.1f} km/h',
                f"{int(self.fr['frame'][index]):04d} / {int(self.fr['frame'][-1]):04d}",{1:'Full pose',2:'Directional only',0:'No full pose'}[status]]
        for k,value in enumerate(values):
            text(d,(40+k*378,121),value,24,EST if k==0 else WHITE,True)
        # Fixed +/-40 cm metric offset inset. No magnification of map traces.
        x,y,w,h=DETAIL;cx=x+165;cy=y+143;mask=(self.t<=t)&(self.t>=t-5)
        residual=(self.estimate[mask,:2]-self.truth[mask,:2])*100
        pixels=np.column_stack((cx+residual[:,0]*2,cy-residual[:,1]*2))
        if len(pixels)>1:d.line([tuple(v) for v in pixels],fill='#9c773f',width=1)
        ex,ey=cx+delta[0]*200,cy-delta[1]*200
        d.line((cx,cy,ex,ey),fill=EST,width=2)
        d.ellipse((cx-5,cy-5,cx+5,cy+5),fill=TRUTH)
        d.ellipse((ex-5,ey-5,ex+5,ey+5),fill=EST,outline=WHITE,width=1)
        # Timeline uses original localization errors, revealed only after acquisition.
        x,y,w,h=TIMELINE;mask=self.t<=t
        values=np.column_stack((x+66+self.t[mask]/120*(w-84),y+122-self.tr['positionErrorM'][mask]*180))
        if len(values)>1:d.line([tuple(v) for v in values],fill=EST,width=2)
        px=x+66+t/120*(w-84);d.line((px,y+45,px,y+122),fill=WHITE,width=1)
        d.ellipse((px-4,y+122-error*1.8-4,px+4,y+122-error*1.8+4),fill=WHITE)
        return im,index,age


def validate_video(out, fps):
    source=np.genfromtxt(out/'frames.csv',delimiter=',',names=True)
    frame_time=source['time'];frame_ids=source['frame'].astype(int)
    frames=int(np.ceil(frame_time[-1]*fps))+1
    timestamps=np.minimum(np.arange(frames)/fps,frame_time[-1])
    used=np.searchsorted(frame_time,timestamps,side='right')
    ages=timestamps-frame_time[used-1]
    maximum_source_gap=float(np.max(np.diff(frame_time)))
    assert set(used)==set(range(1,len(source)+1)), 'Every covered perception frame must appear.'
    assert min(ages)>=-1e-10 and max(ages)<maximum_source_gap+1e-10, 'Perception clock mismatch.'
    video=out/'mncav_localization.mp4'
    probe_command=['ffprobe','-v','error','-count_frames','-show_streams','-show_format','-of','json',str(video)]
    probe=json.loads(subprocess.check_output(probe_command,text=True));(out/'ffprobe.json').write_text(json.dumps(probe,indent=2)+'\n')
    stream=probe['streams'][0];assert int(stream['nb_read_frames'])==frames
    assert stream['width']==W and stream['height']==H and stream['pix_fmt']=='yuv420p'
    assert stream['r_frame_rate']==f'{fps}/1'
    assert abs(float(probe['format']['duration'])-frame_time[-1])<2/fps
    with (out/'decode.log').open('w') as log:
        subprocess.run(['ffmpeg','-v','error','-i',str(video),'-f','null','-'],check=True,stderr=log)
    with video.open('rb') as stream_bytes:digest=hashlib.file_digest(stream_bytes,'sha256').hexdigest()
    table=np.column_stack((np.arange(frames),timestamps,frame_ids[used-1],ages,used,ages))
    np.savetxt(out/'video_frame_clock.csv',table,delimiter=',',header='videoFrame,replayTime,perceptionFrame,perceptionAgeSeconds,localizationSample,localizationAgeSeconds',comments='',fmt=['%d','%.12f','%d','%.12f','%d','%.12f'])
    report={'video':str(video.relative_to(ROOT)) if video.is_relative_to(ROOT) else str(video),
            'sizeBytes':video.stat().st_size,'sha256':digest,
            'width':W,'height':H,'fps':fps,'encodedFrames':frames,'durationSeconds':float(probe['format']['duration']),
            'sourceDurationSeconds':float(frame_time[-1]),'sourcePerceptionFrames':len(set(used)),
            'maximumSourceFrameGapSeconds':maximum_source_gap,
            'maximumPerceptionAgeSeconds':float(max(ages)),'minimumPerceptionAgeSeconds':float(min(ages)),
            'allPerceptionFramesShown':True,'futurePerceptionFrameUsed':False,'fullDecodePassed':True,
            'localizationSamples':len(source),'displayStateInterpolation':False,'displayPolicy':'Hold latest actual synchronized state and perception',
            'playbackSpeed':1,'estimatorRerun':False,'imageryRole':'Context only; USGS/USDA aerial orthoimagery'}
    (out/'video_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/localization_video_20260917')
    parser.add_argument('--preview-only',action='store_true')
    parser.add_argument('--validate-only',action='store_true')
    parser.add_argument('--fps',type=int,default=30)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if args.validate_only:
        validate_video(out,args.fps);return
    renderer=Renderer(out);previews=[]
    for t in renderer.preview_times:
        im,_,_=renderer.render(t);path=out/f'preview_{t:06.2f}s.png';im.save(path);previews.append(im.resize((960,540),Image.Resampling.LANCZOS))
    sheet=Image.new('RGB',(1920,1620),BG)
    for k,im in enumerate(previews):sheet.paste(im,((k%2)*960,(k//2)*540))
    sheet.save(out/'storyboard.jpg',quality=95)
    print('Preview storyboard ready.',flush=True)
    if args.preview_only:return
    assert args.fps>=15
    video=out/'mncav_localization.mp4';frames=int(np.ceil(renderer.duration*args.fps))+1
    command=['ffmpeg','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(args.fps),'-i','-',
             '-an','-c:v','libx264','-threads','4','-preset','medium','-crf','18','-pix_fmt','yuv420p',
             '-movflags','+faststart',str(video)]
    timer=time.monotonic();used=[];ages=[];timestamps=[]
    with (out/'encoding.log').open('w') as log:
        process=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=log)
        try:
            for k in range(frames):
                t=min(k/args.fps,renderer.duration);im,index,age=renderer.render(t)
                process.stdin.write(im.tobytes());used.append(index+1);ages.append(age);timestamps.append(t)
                if k%300==0 or k==frames-1:
                    state={'renderedFrames':k+1,'totalFrames':frames,'replayTime':t,'elapsedSeconds':time.monotonic()-timer}
                    (out/'render_progress.json').write_text(json.dumps(state)+'\n');print(state,flush=True)
        finally:
            process.stdin.close()
        assert process.wait()==0,'Video encoding failed; see encoding.log.'
    assert set(used)==set(range(1,len(renderer.fr)+1))
    validate_video(out,args.fps)


if __name__=='__main__':
    main()

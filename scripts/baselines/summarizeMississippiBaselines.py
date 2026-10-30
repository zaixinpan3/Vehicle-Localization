"""Render compact paper tables from completed paired baseline summaries."""
import csv
import statistics
from pathlib import Path
import argparse
LABELS={'proposed':'Proposed','overlap_ekf':'Overlap EKF','overlap_iekf':'Overlap iterated EKF','gnss_ekf':'GNSS EKF','no_lidar':'Without LiDAR','no_acceleration':'Without acceleration outputs','zero_lateral':'Zero lateral velocity','uniform_evidence':'Without evidence factors'}
def main():
 p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,default=Path('output/mississippi_baselines_20261030'));a=p.parse_args();rows=[]
 for drive in [1,2]:
  with (a.folder/f'drive{drive}_metrics.csv').open() as f:current=list(csv.DictReader(f))
  assert len(current)==256, f'Drive {drive} is incomplete: {len(current)} rows'
  rows+=current
 with (a.folder/'combined_metrics.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
 out=['\\subsubsection{Current paired results}\\mbox{}\\par',
 '\\begin{table*}[t]\\centering\\caption{Current same-input Mississippi comparisons. Every query frame is scored. Position entries are RMSE / maximum in metres; heading is RMSE in degrees. Runtime excludes common offline preparation.}\\label{tab:current_mississippi_baselines}',
 '\\small\\begin{tabular}{llrrrr}\\toprule Sequence & Method & Both sensors & GNSS gap & LiDAR gap & Yaw (both)\\\\\\midrule']
 def find(d,s,m):return next(r for r in rows if r['drive']==str(d) and r['scenario']==s and r['method']==m and r['window']=='all')
 def metric(r):return f"{float(r['rmseM']):.3f} / {float(r['maximumM']):.3f}"
 for d in [1,2]:
  for m in list(LABELS)[:5]:
   b=find(d,'both',m);out.append(f"{d} & {LABELS[m]} & {metric(b)} & {metric(find(d,'gnss_loss',m))} & {metric(find(d,'lidar_loss',m))} & {float(b['headingRmseDeg']):.3f}\\\\")
  if d==1:out.append('\\midrule')
 out+=['\\bottomrule\\end{tabular}\\end{table*}',
 '\\begin{table*}[t]\\centering\\caption{Current ablations and complete GNSS withdrawal. Entries are position RMSE / maximum in metres, including initialization.}\\label{tab:current_mississippi_ablations}',
 '\\small\\begin{tabular}{llrr}\\toprule Sequence & Method & Both sensors & No GNSS\\\\\\midrule']
 for d in [1,2]:
  for m in ['proposed','no_lidar','no_acceleration','zero_lateral','uniform_evidence']:
   out.append(f"{d} & {LABELS[m]} & {metric(find(d,'both',m))} & {metric(find(d,'no_gnss',m))}\\\\")
  if d==1:out.append('\\midrule')
 out+=['\\bottomrule\\end{tabular}\\end{table*}']
 out+=['\\begin{table*}[t]\\centering\\caption{Intermittent availability. Random loss entries give the median position RMSE and largest position maximum across three fixed masks. These repeated masks are not independent routes.}\\label{tab:current_mississippi_dropout}',
 '\\small\\begin{tabular}{llrrr}\\toprule Sequence & Method & Alternating & 80\\% loss & Both (ms/frame)\\\\\\midrule']
 for d in [1,2]:
  for m in ['proposed','overlap_ekf','overlap_iekf','no_lidar']:
   random=[find(d,f'random80_{j}',m) for j in [1,2,3]];b=find(d,'both',m)
   med=statistics.median(float(r['rmseM']) for r in random);peak=max(float(r['maximumM']) for r in random)
   ms=1000*float(b['runtimeS'])/int(b['frames'])
   out.append(f"{d} & {LABELS[m]} & {metric(find(d,'alternating',m))} & {med:.3f} / {peak:.3f} & {ms:.2f}\\\\")
  if d==1:out.append('\\midrule')
 out+=['\\bottomrule\\end{tabular}\\end{table*}']
 with (a.folder/'nominal_sensitivity.csv').open() as f:nominal=list(csv.DictReader(f))
 assert len(nominal)==12,'Nominal-tuning sensitivity is incomplete'
 out+=['\\begin{table*}[t]\\centering\\caption{Nominal-only calibration sensitivity, using parameters selected before test scoring from the same calibration grid. These variants optimize nominal calibration error rather than the three-regime selection used above. Entries are position RMSE / maximum in metres.}\\label{tab:current_mississippi_nominal}',
 '\\small\\begin{tabular}{llrrr}\\toprule Sequence & Method & Both sensors & GNSS blackout & No GNSS\\\\\\midrule']
 for d in [1,2]:
  for m in ['ekf','iekf']:
   selected=[next(r for r in nominal if r['drive']==str(d) and r['method']==m and r['scenario']==s) for s in ['both','gnss_loss','no_gnss']]
   label='EKF nominal tuning' if m=='ekf' else 'Iterated EKF nominal tuning'
   out.append(f"{d} & {label} & {' & '.join(metric(r) for r in selected)}\\\\")
  if d==1:out.append('\\midrule')
 out+=['\\bottomrule\\end{tabular}\\end{table*}']
 with (a.folder/'ndt_metrics.csv').open() as f:ndt=list(csv.DictReader(f))
 with (a.folder/'bessafa_metrics.csv').open() as f:bessafa=list(csv.DictReader(f))
 assert len(ndt)==6 and len(bessafa)==6,'External-family controls incomplete'
 out+=['\\begin{table*}[t]\\centering\\caption{Additional front-end and source-family controls. Entries are position RMSE / maximum in metres. Failed means that the replay hit the declared numerical/speed guard; it is not scored on its surviving prefix. Bessafa has no LiDAR and should be compared with the proposed no-LiDAR row.}\\label{tab:current_mississippi_external}',
 '\\small\\begin{tabular}{llrrr}\\toprule Sequence & Method & Both available & GNSS gap & No GNSS\\\\\\midrule']
 for d in [1,2]:
  for label,data in [('NDT + EKF (adapted)',ndt),('Bessafa LMI member',bessafa)]:
   selected=[next(r for r in data if r['drive']==str(d) and r['scenario']==s) for s in ['both','gnss_loss','no_gnss']]
   values=['Failed' if r.get('complete')=='0' else metric(r) for r in selected]
   out.append(f"{d} & {label} & {' & '.join(values)}\\\\")
  if d==1:out.append('\\midrule')
 out+=['\\bottomrule\\end{tabular}\\end{table*}']
 Path('paper/mississippi_baseline_results.tex').write_text('\n'.join(out)+'\n')
 for d in [1,2]:
  b=find(d,'both','proposed');print('DRIVE',d,'frames',b['frames'],'proposed',metric(b),'EKF',metric(find(d,'both','overlap_ekf')),'IEKF',metric(find(d,'both','overlap_iekf')))
 print('Completed runs',len(rows)//4,'nonfinite runs',sum(r['complete']!='1' for r in rows if r['window']=='all'))
if __name__=='__main__':main()

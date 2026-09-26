// Bounded shaft modes on original pillar points; mirrors findPillarShaftModes.m.
namespace pillar_shaft {
using pole_subset::Point;
using pole_subset::Axis;
using pole_subset::balancedAxis;
using pole_subset::distance;
using pole_subset::quantile;

struct FitScratch {
    std::vector<Point> sorted;
    std::vector<double> x,y,z,weights;
    std::vector<mwSize> starts;
};

double median(std::vector<double>& values) {
    const mwSize hi=values.size()/2;
    std::nth_element(values.begin(),values.begin()+hi,values.end());
    if (values.size()%2) return values[hi];
    const double lo=*std::max_element(values.begin(),values.begin()+hi);
    // Use the same interpolation and arithmetic as quantile(values,.5).
    return lo+.5*(values[hi]-lo);
}

Axis fitAxis(const std::vector<Point>& points,bool vertical,FitScratch& scratch) {
    auto& z=scratch.z;z.clear();
    if (vertical) {
        auto& x=scratch.x;auto& y=scratch.y;x.clear();y.clear();
        for (const auto& p:points) {x.push_back(p.x);y.push_back(p.y);z.push_back(p.z);}
        return {median(x),median(y),median(z),0,0};
    }
    const auto byHeight=[](const Point& a,const Point& b){return a.z<b.z;};
    const std::vector<Point>* sorted=&points;
    if (!std::is_sorted(points.begin(),points.end(),byHeight)) {
        scratch.sorted=points;
        std::stable_sort(scratch.sorted.begin(),scratch.sorted.end(),byHeight);
        sorted=&scratch.sorted;
    }
    auto& starts=scratch.starts;starts.clear();
    for (mwSize i=0;i<sorted->size();++i) {
        if (i==0 || (*sorted)[i].z!=(*sorted)[i-1].z) {z.push_back((*sorted)[i].z);starts.push_back(i);}
    }
    starts.push_back(sorted->size());
    Axis a{0,0,0,0,0};
    if (z.size()==1) {
        for (const auto& p:points) {a.x+=p.x;a.y+=p.y;}
        a.x/=points.size();a.y/=points.size();a.z=z.front();return a;
    }
    // Preserve the height-balanced fit's original accumulation order.
    const double total=z.back()-z.front();auto& weights=scratch.weights;weights.resize(z.size());
    for (mwSize g=0;g<z.size();++g) {
        weights[g]=((g?z[g]-z[g-1]:0)+(g+1<z.size()?z[g+1]-z[g]:0))/2;
        weights[g]/=total*static_cast<double>(starts[g+1]-starts[g]);
        for (mwSize i=starts[g];i<starts[g+1];++i) {
            a.x+=weights[g]*(*sorted)[i].x;a.y+=weights[g]*(*sorted)[i].y;a.z+=weights[g]*(*sorted)[i].z;
        }
    }
    double variance=0;
    for (mwSize g=0;g<z.size();++g) for (mwSize i=starts[g];i<starts[g+1];++i) {
        const double dz=(*sorted)[i].z-a.z,w=weights[g];
        variance+=w*dz*dz;a.sx+=w*dz*((*sorted)[i].x-a.x);a.sy+=w*dz*((*sorted)[i].y-a.y);
    }
    variance=std::max(variance,std::numeric_limits<double>::epsilon());a.sx/=variance;a.sy/=variance;return a;
}

struct AxisSample { Point point;double x,y,distance; };
struct IntervalScratch {
    std::vector<mwSize> nearby,core,lowIndex,highIndex,nextOwned,previousOwned,endpoints;
    std::vector<double> height,prefix,square,owner,coverage;
};

void intervals(const std::vector<AxisSample>& samples,const Axis& axis,double radius,
               const double* p,double* output,mwSize row,mwSize count,IntervalScratch& scratch) {
    auto& nearby=scratch.nearby;auto& core=scratch.core;auto& height=scratch.height;
    nearby.clear();core.clear();height.clear();
    for (mwSize k=0;k<samples.size();++k) {
        const auto& sample=samples[k];
        if (sample.distance>2*radius*(1+1e-14)) continue;
        if (sample.distance<=radius) core.push_back(nearby.size());
        nearby.push_back(k);height.push_back(sample.point.z);
    }
    if (core.size()<p[5]) return;
    const mwSize n=nearby.size();
    const auto point=[&](mwSize k)->const Point& {return samples[nearby[k]].point;};
    const double diagonal=std::sqrt(.5),r2=radius*radius;
    const double directions[8][2]={{1,0},{0,1},{diagonal,diagonal},{diagonal,-diagonal},
                                 {-1,0},{0,-1},{-diagonal,-diagonal},{-diagonal,diagonal}};
    auto& prefix=scratch.prefix;bool prefixReady=false;
    const auto buildPrefix=[&]() {
        prefix.resize((n+1)*10);std::fill(prefix.begin(),prefix.begin()+10,0);
        for (mwSize k=0;k<n;++k) {
            const auto& sample=samples[nearby[k]];
            prefix[(k+1)*10]=prefix[k*10]+(sample.distance<=radius);
            prefix[(k+1)*10+9]=prefix[k*10+9]+(sample.distance<=2*radius);
            for (int d=0;d<8;++d) {
                const double xx=sample.x-radius*directions[d][0],yy=sample.y-radius*directions[d][1];
                prefix[(k+1)*10+d+1]=prefix[k*10+d+1]+(xx*xx+yy*yy<=r2);
            }
        }
        prefixReady=true;
    };
    auto& square=scratch.square;auto& owner=scratch.owner;auto& coveragePrefix=scratch.coverage;
    square.resize(core.size()+1);owner.resize(core.size()+1);coveragePrefix.resize(core.size());
    square[0]=0;owner[0]=0;coveragePrefix[0]=0;
    auto& lowIndex=scratch.lowIndex;auto& highIndex=scratch.highIndex;
    lowIndex.resize(core.size());highIndex.resize(core.size());
    auto& nextOwned=scratch.nextOwned;auto& previousOwned=scratch.previousOwned;
    nextOwned.resize(core.size());previousOwned.resize(core.size());
    mwSize low=0,high=0,previous=0;
    for (mwSize k=0;k<core.size();++k) {
        const auto& sample=samples[nearby[core[k]]];
        square[k+1]=square[k]+sample.distance*sample.distance;
        owner[k+1]=owner[k]+sample.point.owned;
        if (sample.point.owned) previous=k;previousOwned[k]=previous;
        while (low<n && height[low]<sample.point.z) ++low;
        if (high<low) high=low;while (high<n && height[high]<=sample.point.z) ++high;
        lowIndex[k]=low;highIndex[k]=high;
        if (k>0) coveragePrefix[k]=coveragePrefix[k-1]+std::min(sample.point.z-point(core[k-1]).z,.20);
    }
    mwSize next=core.size()-1;
    for (mwSize k=core.size();k>0;) {--k;if (point(core[k]).owned) next=k;nextOwned[k]=next;}
    const double gapLimit=p[11]+p[12]*std::hypot(axis.x,axis.y);
    mwSize first=0;
    while (first<core.size()) {
        mwSize end=first+1;while (end<core.size() && point(core[end]).z-point(core[end-1]).z<=gapLimit) ++end;
        const mwSize length=end-first;
        if (length>=p[5]) {
            const mwSize limit=std::min(length,static_cast<mwSize>(p[16]));
            auto& endpoints=scratch.endpoints;endpoints.clear();
            for (mwSize k=0;k<limit;++k) {
                const mwSize at=first+static_cast<mwSize>(std::round(static_cast<double>(length-1)*k/(limit-1)));
                if (endpoints.empty() || endpoints.back()!=at) endpoints.push_back(at);
            }
            for (auto a:endpoints) for (auto b:endpoints) {
                if (b<a || b-a+1<p[5]) continue;
                const double lo=point(core[a]).z,hi=point(core[b]).z,span=hi-lo,owned=owner[b+1]-owner[a];
                if (span<p[8] || owned<p[6]) continue;
                const double ownHeight=point(core[previousOwned[b]]).z-point(core[nextOwned[a]]).z;
                if (ownHeight<p[7]) continue;
                const double radial=std::sqrt(std::max(0.0,(square[b+1]-square[a])/(b-a+1)));
                if (radial>std::min(p[13],p[26]*radius)) continue;
                // Quantiles are interpolated directly in this sorted subrange.
                const auto value=[&](double fraction) {
                    const double at=std::clamp((b-a+1)*fraction-.5,0.0,static_cast<double>(b-a));
                    const mwSize lower=static_cast<mwSize>(std::floor(at)),upper=static_cast<mwSize>(std::ceil(at));
                    return point(core[a+lower]).z+(at-lower)*(point(core[a+upper]).z-point(core[a+lower]).z);
                };
                const double robust=value(.95)-value(.05);if (robust<p[9]) continue;
                const double coverage=(coveragePrefix[b]-coveragePrefix[a])/span;
                const double geometric=std::min(1.0,robust/p[18])*std::sqrt(std::min(1.0,(b-a+1)/p[19]))*
                    std::exp(-std::pow(radial/p[13],2))*std::sqrt(coverage);
                if (geometric<p[17] || geometric<=output[row]+1e-12) continue;
                // Most hypotheses fail the cheap geometric bounds. Probe
                // disks only when some interval can improve this pillar.
                if (!prefixReady) buildPrefix();
                const mwSize low=lowIndex[a],high=highIndex[b];
                const double center=prefix[high*10]-prefix[low*10];double side=0;
                for (int k=1;k<9;++k) side=std::max(side,prefix[high*10+k]-prefix[low*10+k]);
                const double peak=center/std::max(side,1.0),excess=center-side;
                if (peak<p[14] || excess<p[15]) continue;
                const double outer=prefix[high*10+9]-prefix[low*10+9];
                const double radialSignificance=(center-.25*outer)/std::sqrt(std::max(.1875*outer,1.0));
                if (radialSignificance<p[25]) continue;
                const mwSize middle=std::upper_bound(height.begin(),height.end(),.5*(lo+hi))-height.begin();
                const mwSize edges[3]={low,middle,high};bool persistent=true;
                for (int half=0;half<2;++half) {
                    const mwSize a=edges[half]*10,b=edges[half+1]*10;
                    const double inner=prefix[b]-prefix[a],surround=prefix[b+9]-prefix[a+9];double sideHalf=0;
                    for (int k=1;k<9;++k) sideHalf=std::max(sideHalf,prefix[b+k]-prefix[a+k]);
                    const double significant=(inner-.25*surround)/std::sqrt(std::max(.1875*surround,1.0));
                    if (significant<p[27] || inner/std::max(sideHalf,1.0)<p[28]) {persistent=false;break;}
                }
                if (!persistent) continue;
                const double significance=excess/std::sqrt(std::max(center+side,1.0));
                const double score=geometric*std::min(1.0,(peak-1)/.6);
                if (score<p[17] || score<=output[row]+1e-12) continue;
                double gap=0;for (mwSize k=a+1;k<=b;++k) gap=std::max(gap,point(core[k]).z-point(core[k-1]).z);
                const double values[21]={score,static_cast<double>(b-a+1),owned,span,robust,radial,gap,peak,
                    radius,lo,hi,axis.z,ownHeight,peak,axis.x,axis.y,axis.sx,axis.sy,excess,significance,coverage};
                for (int k=0;k<21;++k) output[row+k*count]=values[k];
                output[row+22*count]=radialSignificance;
            }
        }
        first=end;
    }
}

void run(int nlhs,mxArray** out,int nrhs,const mxArray** in) {
    require(nrhs==12 && nlhs==1,"Shaft modes need eleven inputs and one output.");
    const mwSize N=mxGetM(in[1]),P=mxGetNumberOfElements(in[3]),R=mxGetNumberOfElements(in[7]),F=mxGetNumberOfElements(in[8]);
    require(mxGetN(in[1])==3,"Points must be an XYZ matrix.");
    array(in[1],mxDOUBLE_CLASS,N*3);array(in[2],mxDOUBLE_CLASS,N);array(in[3],mxDOUBLE_CLASS,P);
    array(in[4],mxDOUBLE_CLASS,2);array(in[5],mxDOUBLE_CLASS,2);array(in[6],mxDOUBLE_CLASS,29);
    array(in[7],mxDOUBLE_CLASS,R);array(in[8],mxDOUBLE_CLASS,F);array(in[9],mxLOGICAL_CLASS,P);
    array(in[10],mxDOUBLE_CLASS,P);const double* thresholds=mxGetDoubles(in[10]);
    array(in[11],mxDOUBLE_CLASS,1);const double requestedThreads=*mxGetDoubles(in[11]);
    require(std::isfinite(requestedThreads) && requestedThreads>=1 && requestedThreads<=64 && requestedThreads==std::floor(requestedThreads),"Invalid shaft thread count.");
    for (mwSize k=0;k<P;++k) require(std::isfinite(thresholds[k]) && thresholds[k]>=0,"Invalid per-pillar threshold.");
    const double *xyz=mxGetDoubles(in[1]),*group=mxGetDoubles(in[2]),*ids=mxGetDoubles(in[3]);
    const double *dims=mxGetDoubles(in[4]),*spacing=mxGetDoubles(in[5]),*p=mxGetDoubles(in[6]);
    const double *radii=mxGetDoubles(in[7]),*fits=mxGetDoubles(in[8]);const mxLogical* evaluate=mxGetLogicals(in[9]);
    for (int k=0;k<2;++k) require(std::isfinite(dims[k]) && dims[k]>=1 && dims[k]<=2147483647 && dims[k]==std::floor(dims[k]) && std::isfinite(spacing[k]) && spacing[k]>0,"Invalid shaft geometry.");
    require(dims[0]*dims[1]<static_cast<double>(std::numeric_limits<mwSignedIndex>::max()),"Shaft raster is too large.");
    for (int k=0;k<29;++k) require(std::isfinite(p[k]) && p[k]>=0,"Invalid shaft parameters.");
    require(p[0]>=1 && p[0]<=256 && p[0]==std::floor(p[0]) && p[3]>=1 && p[3]<=256 && p[3]==std::floor(p[3]),"Invalid shaft search limits.");
    require(p[5]>=3 && p[6]>=2 && p[8]>0 && p[10]<90 && p[13]>0 && p[16]>=2 && p[16]<=256 && p[16]==std::floor(p[16]) && p[18]>0 && p[19]>0 && p[2]>0,"Invalid shaft support limits.");
    require(R>0 && F>0 && F<=16 && p[22]>=1 && p[22]<=F,"Invalid shaft radii or consensus requirement.");
    for (mwSize k=0;k<R;++k) require(std::isfinite(radii[k]) && radii[k]>0,"Invalid shaft radius.");
    for (mwSize k=0;k<F;++k) require(std::isfinite(fits[k]) && fits[k]>0,"Invalid fit radius.");
    const mwSize rows=static_cast<mwSize>(dims[0]),cols=static_cast<mwSize>(dims[1]);
    std::vector<std::vector<mwSize>> members(P);std::vector<mwSignedIndex> lookup(rows*cols,-1);
    for (mwSize k=0;k<P;++k) {require(index(ids[k],rows*cols),"Invalid pillar index.");const mwSize id=static_cast<mwSize>(ids[k])-1;require(lookup[id]<0,"Pillar indices must be unique.");lookup[id]=k;}
    for (mwSize k=0;k<N;++k) {
        require(index(group[k],P) && std::isfinite(xyz[k]) && std::isfinite(xyz[k+N]) && std::isfinite(xyz[k+2*N]),"Invalid point or owner.");
        members[static_cast<mwSize>(group[k])-1].push_back(k);
    }
    out[0]=mxCreateDoubleMatrix(P,23,mxREAL);double* output=mxGetDoubles(out[0]);
    std::fill(output+14*P,output+16*P,std::numeric_limits<double>::quiet_NaN());
    const int reachX=static_cast<int>(std::min(dims[1]-1,std::ceil(p[4]/spacing[0]))),reachY=static_cast<int>(std::min(dims[0]-1,std::ceil(p[4]/spacing[1])));
    const double maximumSlope=std::tan(p[10]*std::acos(-1.0)/180);
    const double maximumRadius=*std::max_element(radii,radii+R);
    std::vector<mwSize> eligible;
    for (mwSize j=0;j<P;++j) if (evaluate[j] && members[j].size()>=p[6]) eligible.push_back(j);
    if (eligible.empty()) return;
    const mwSize hardware=std::max(1u,std::thread::hardware_concurrency());
    // Small calls remain serial. Each worker owns all scratch and writes
    // only its assigned output rows; no MATLAB API is called by a worker.
    const mwSize workers=std::min({static_cast<mwSize>(requestedThreads),hardware,std::max<mwSize>(1,eligible.size()/8)});
    std::atomic<mwSize> nextColumn{0};std::atomic<bool> failed{false};
    std::vector<std::exception_ptr> errors(workers);
    const auto process=[&](mwSize worker) {
        try {
            FitScratch fitScratch;IntervalScratch intervalScratch;
            std::vector<AxisSample> samples;
            while (!failed.load(std::memory_order_relaxed)) {
                const mwSize task=nextColumn.fetch_add(1,std::memory_order_relaxed);
                if (task>=eligible.size()) break;
                const mwSize j=eligible[task];
                std::vector<Point> own;double bottom=std::numeric_limits<double>::infinity(),top=-bottom;
                for (auto i:members[j]) {own.push_back({xyz[i],xyz[i+N],xyz[i+2*N],true});bottom=std::min(bottom,xyz[i+2*N]);top=std::max(top,xyz[i+2*N]);}
                if (top-bottom<p[7]) continue;
                std::stable_sort(own.begin(),own.end(),[](const Point& a,const Point& b){return a.x!=b.x?a.x<b.x:(a.y!=b.y?a.y<b.y:a.z<b.z);});
                const mwSignedIndex row=(static_cast<mwSize>(ids[j])-1)%rows,col=(static_cast<mwSize>(ids[j])-1)/rows;
                std::vector<Point> q;
                for (mwSignedIndex c=std::max<mwSignedIndex>(0,col-reachX);c<=std::min<mwSignedIndex>(cols-1,col+reachX);++c)
                    for (mwSignedIndex r=std::max<mwSignedIndex>(0,row-reachY);r<=std::min<mwSignedIndex>(rows-1,row+reachY);++r) {
                        const mwSignedIndex neighbor=lookup[r+c*rows];if (neighbor<0) continue;
                        for (auto i:members[neighbor]) q.push_back({xyz[i],xyz[i+N],xyz[i+2*N],static_cast<mwSize>(neighbor)==j});
                    }
                std::stable_sort(q.begin(),q.end(),[](const Point& a,const Point& b){return a.z<b.z;});
                std::vector<Point> seeds{own.front()};std::vector<double> nearest(own.size(),std::numeric_limits<double>::infinity());
                for (int k=1;k<static_cast<int>(p[0]);++k) {
                    const auto& last=seeds.back();double far=-1;mwSize winner=0;
                    for (mwSize i=0;i<own.size();++i) {const double dx=own[i].x-last.x,dy=own[i].y-last.y;nearest[i]=std::min(nearest[i],dx*dx+dy*dy);if (nearest[i]>far) {far=nearest[i];winner=i;}}
                    if (far<p[1]*p[1]) break;seeds.push_back(own[winner]);
                }
                std::vector<Axis> axes;std::vector<unsigned int> scaleMasks;
                for (const auto& seed:seeds) for (mwSize f=0;f<F;++f) {
                    const double fit=fits[f];std::vector<Point> initial;double lo=std::numeric_limits<double>::infinity(),hi=-lo;
                    for (const auto& a:own) if (std::hypot(a.x-seed.x,a.y-seed.y)<=fit) {initial.push_back(a);lo=std::min(lo,a.z);hi=std::max(hi,a.z);}
                    if (initial.size()<p[6] || hi-lo<p[7]) continue;
                    std::vector<std::pair<double,double>> windows;
                    for (int w=0;w<=static_cast<int>(p[3]);++w) {
                        double low=lo,high=hi;
                        if (w>0) {low=lo+(std::max(lo,hi-p[2])-lo)*(p[3]==1?1:static_cast<double>(w-1)/(p[3]-1));high=std::min(low+p[2],hi);}
                        const auto window=std::make_pair(low,high);if (std::find(windows.begin(),windows.end(),window)!=windows.end()) continue;windows.push_back(window);
                        std::vector<Point> fitting;double fitLo=std::numeric_limits<double>::infinity(),fitHi=-fitLo;
                        for (const auto& a:initial) if (a.z>=low && a.z<=high) {fitting.push_back(a);fitLo=std::min(fitLo,a.z);fitHi=std::max(fitHi,a.z);}
                        if (fitting.size()<p[6] || fitHi-fitLo<p[7]) continue;
                        const auto initialFit=fitting;
                        const auto begin=std::lower_bound(q.begin(),q.end(),low,[](const Point& a,double z){return a.z<z;});
                        const auto end=std::upper_bound(begin,q.end(),high,[](double z,const Point& a){return z<a.z;});
                        for (int vertical=0;vertical<=static_cast<int>(p[24]>0);++vertical) {
                            Axis axis=fitAxis(initialFit,vertical,fitScratch);if (std::hypot(axis.sx,axis.sy)>maximumSlope) continue;
                            for (int iteration=0;iteration<2;++iteration) {
                                fitting.clear();for (auto at=begin;at!=end;++at) if (distance(*at,axis)<=fit) fitting.push_back(*at);
                                if (fitting.size()<p[5]) break;Axis candidate=fitAxis(fitting,vertical,fitScratch);if (std::hypot(candidate.sx,candidate.sy)>maximumSlope) break;axis=candidate;
                            }
                            bool duplicate=false;
                            for (mwSize a=0;a<axes.size();++a) {
                                const auto& old=axes[a];
                                if (std::hypot(old.x+(axis.z-old.z)*old.sx-axis.x,old.y+(axis.z-old.z)*old.sy-axis.y)<=p[20] && std::hypot(old.sx-axis.sx,old.sy-axis.sy)<=p[21]) {duplicate=true;scaleMasks[a]|=(1u<<f);break;}
                            }
                            if (duplicate) continue;axes.push_back(axis);scaleMasks.push_back(1u<<f);
                        }
                    }
                }
                for (mwSize a=0;a<axes.size();++a) {
                    unsigned int scaleCount=0;for (mwSize f=0;f<F;++f) scaleCount+=(scaleMasks[a]>>f)&1u;
                    double settings[29];std::copy(p,p+29,settings);
                    settings[17]=std::max(p[17],thresholds[j]);
                    if (scaleCount<p[22]) settings[17]=std::max(settings[17],p[23]);
                    // Geometry is shared by every tested radius. Points farther than
                    // twice the largest radius cannot enter any disk probe.
                    samples.clear();const auto& axis=axes[a];
                    for (const auto& point:q) {
                        const double x=point.x-axis.x-(point.z-axis.z)*axis.sx;
                        const double y=point.y-axis.y-(point.z-axis.z)*axis.sy;
                        const double d=std::hypot(x,y);
                        if (d<=2*maximumRadius*(1+1e-14)) samples.push_back({point,x,y,d});
                    }
                    const double before=output[j];
                    for (mwSize k=0;k<R;++k) intervals(samples,axes[a],radii[k],settings,output,j,P,intervalScratch);
                    if (output[j]>before+1e-12) output[j+21*P]=scaleCount;
                }
            }
        } catch (...) {
            errors[worker]=std::current_exception();failed.store(true,std::memory_order_relaxed);
        }
    };
    std::vector<std::thread> threads;threads.reserve(workers-1);
    try {
        for (mwSize worker=1;worker<workers;++worker) threads.emplace_back(process,worker);
    } catch (...) {
        failed.store(true,std::memory_order_relaxed);
        for (auto& thread:threads) thread.join();
        throw;
    }
    process(0);
    for (auto& thread:threads) thread.join();
    for (const auto& error:errors) if (error) std::rethrow_exception(error);
}
} // namespace pillar_shaft

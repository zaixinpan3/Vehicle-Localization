// Existential compact-shaft evidence. Included inside the kernel namespace.
// This mirrors findPillarPoleSubsets.m without retaining frame state.
namespace pole_subset {
struct Point { double x,y,z; bool owned; };
struct Axis { double x,y,z,sx,sy; };
Axis balancedAxis(const std::vector<Point>& points) {
    std::vector<Point> sorted=points;
    std::stable_sort(sorted.begin(),sorted.end(),[](const Point& a,const Point& b){return a.z<b.z;});
    std::vector<double> z;
    std::vector<mwSize> starts;
    for (mwSize i=0;i<sorted.size();++i) {
        if (i==0 || sorted[i].z!=sorted[i-1].z) {z.push_back(sorted[i].z);starts.push_back(i);}
    }
    starts.push_back(sorted.size());
    Axis a{0,0,0,0,0};
    if (z.size()==1) {
        for (const auto& p:points) {a.x+=p.x;a.y+=p.y;}
        a.x/=points.size();a.y/=points.size();a.z=z.front();return a;
    }
    // Equal physical height measure shared among returns at the same height.
    const double total=z.back()-z.front();
    std::vector<double> weights(z.size());
    for (mwSize g=0;g<z.size();++g) {
        weights[g]=((g?z[g]-z[g-1]:0)+(g+1<z.size()?z[g+1]-z[g]:0))/2;
        weights[g]/=total*static_cast<double>(starts[g+1]-starts[g]);
        for (mwSize i=starts[g];i<starts[g+1];++i) {
            a.x+=weights[g]*sorted[i].x;a.y+=weights[g]*sorted[i].y;a.z+=weights[g]*sorted[i].z;
        }
    }
    double variance=0;
    for (mwSize g=0;g<z.size();++g) for (mwSize i=starts[g];i<starts[g+1];++i) {
        const double dz=sorted[i].z-a.z,w=weights[g];
        variance+=w*dz*dz;a.sx+=w*dz*(sorted[i].x-a.x);a.sy+=w*dz*(sorted[i].y-a.y);
    }
    variance=std::max(variance,std::numeric_limits<double>::epsilon());a.sx/=variance;a.sy/=variance;return a;
}
double distance(const Point& p,const Axis& a) {
    return std::hypot(p.x-a.x-(p.z-a.z)*a.sx,p.y-a.y-(p.z-a.z)*a.sy);
}
double quantile(const std::vector<double>& z,double fraction) {
    const double at=std::clamp(z.size()*fraction-0.5,0.0,static_cast<double>(z.size()-1));
    const mwSize lo=static_cast<mwSize>(std::floor(at)),hi=static_cast<mwSize>(std::ceil(at));
    return z[lo]+(at-lo)*(z[hi]-z[lo]);
}
double windowCount(const std::vector<double>& sorted,double at,double half) {
    double lo=at-half;const double scale=std::max(1.0,std::abs(lo));
    lo-=4*(std::nextafter(scale,std::numeric_limits<double>::infinity())-scale);
    return std::upper_bound(sorted.begin(),sorted.end(),at+half)-std::upper_bound(sorted.begin(),sorted.end(),lo);
}
double peakContrast(const std::vector<Point>& q,const Axis& axis,double radius,double lo,double hi) {
    const double d=std::sqrt(0.5);
    const double directions[8][2]={{1,0},{0,1},{d,d},{d,-d},{-1,0},{0,-1},{-d,-d},{-d,d}};
    double center=0,side[8]={};const double r2=radius*radius;
    for (const auto& p:q) {
        if (p.z<lo || p.z>hi) continue;
        const double x=p.x-axis.x-(p.z-axis.z)*axis.sx,y=p.y-axis.y-(p.z-axis.z)*axis.sy;
        center+=x*x+y*y<=r2;
        for (int k=0;k<8;++k) {
            const double xx=x-radius*directions[k][0],yy=y-radius*directions[k][1];
            side[k]+=xx*xx+yy*yy<=r2;
        }
    }
    return center/std::max(1.0,*std::max_element(side,side+8));
}
void run(int nlhs,mxArray** out,int nrhs,const mxArray** in) {
    require(nrhs==9 && nlhs==1,"Pole subsets need eight inputs and one output.");
    const mwSize N=mxGetM(in[1]),P=mxGetNumberOfElements(in[3]),R=mxGetNumberOfElements(in[7]);
    require(mxGetN(in[1])==3,"Points must be an XYZ matrix.");
    array(in[1],mxDOUBLE_CLASS,N*3);array(in[2],mxDOUBLE_CLASS,N);array(in[3],mxDOUBLE_CLASS,P);
    array(in[4],mxDOUBLE_CLASS,2);array(in[5],mxDOUBLE_CLASS,2);array(in[6],mxDOUBLE_CLASS,22);
    array(in[7],mxDOUBLE_CLASS,R);array(in[8],mxLOGICAL_CLASS,P);
    const double *xyz=mxGetDoubles(in[1]),*group=mxGetDoubles(in[2]),*ids=mxGetDoubles(in[3]);
    const double *dims=mxGetDoubles(in[4]),*spacing=mxGetDoubles(in[5]),*p=mxGetDoubles(in[6]),*radii=mxGetDoubles(in[7]);
    const mxLogical* evaluate=mxGetLogicals(in[8]);
    for (int k=0;k<2;++k) require(std::isfinite(dims[k]) && dims[k]>=1 && dims[k]<=std::numeric_limits<int>::max() && dims[k]==std::floor(dims[k]) && std::isfinite(spacing[k]) && spacing[k]>0,"Invalid pole geometry.");
    require(dims[0]*dims[1]<static_cast<double>(std::numeric_limits<mwSignedIndex>::max()),"Pole raster is too large.");
    for (int k=0;k<22;++k) require(std::isfinite(p[k]) && p[k]>=0,"Invalid pole parameters.");
    require(p[0]>=1 && p[0]<=256 && p[0]==std::floor(p[0]) && p[4]>=1 && p[4]<=256 && p[4]==std::floor(p[4]) && p[6]>=3 && p[7]>=2 && p[18]>0 && p[19]>0 && p[20]>0 && p[21]>0,"Invalid pole support limits.");
    require(R>0 && p[11]<90 && p[2]>0 && p[3]>0,"Invalid shaft fit parameters.");
    for (mwSize k=0;k<R;++k) require(std::isfinite(radii[k]) && radii[k]>0,"Invalid shaft radius.");
    const mwSize rows=static_cast<mwSize>(dims[0]),cols=static_cast<mwSize>(dims[1]);
    std::vector<std::vector<mwSize>> members(P);std::vector<mwSignedIndex> lookup(rows*cols,-1);
    for (mwSize k=0;k<P;++k) {
        require(index(ids[k],rows*cols),"Invalid pillar index.");
        const mwSize id=static_cast<mwSize>(ids[k])-1;
        require(lookup[id]<0,"Pillar indices must be unique.");lookup[id]=k;
    }
    for (mwSize k=0;k<N;++k) {
        require(index(group[k],P) && std::isfinite(xyz[k]) && std::isfinite(xyz[k+N]) && std::isfinite(xyz[k+2*N]),"Invalid point or owner.");
        members[static_cast<mwSize>(group[k])-1].push_back(k);
    }
    out[0]=mxCreateDoubleMatrix(P,18,mxREAL);double* output=mxGetDoubles(out[0]);
    std::fill(output+14*P,output+16*P,std::numeric_limits<double>::quiet_NaN());
    const int reachX=static_cast<int>(std::min(dims[1]-1,std::ceil(p[5]/spacing[0])));
    const int reachY=static_cast<int>(std::min(dims[0]-1,std::ceil(p[5]/spacing[1])));
    const double maxSlope=std::tan(p[11]*std::acos(-1.0)/180);
    for (mwSize j=0;j<P;++j) {
        if (!evaluate[j] || members[j].size()<p[7]) continue;
        std::vector<Point> own;
        double bottom=std::numeric_limits<double>::infinity(),top=-bottom;
        for (auto i:members[j]) {own.push_back({xyz[i],xyz[i+N],xyz[i+2*N],true});bottom=std::min(bottom,xyz[i+2*N]);top=std::max(top,xyz[i+2*N]);}
        if (top-bottom<p[8]) continue;
        std::stable_sort(own.begin(),own.end(),[](const Point& a,const Point& b){return a.x!=b.x?a.x<b.x:(a.y!=b.y?a.y<b.y:a.z<b.z);});
        const mwSignedIndex row=(static_cast<mwSize>(ids[j])-1)%rows,col=(static_cast<mwSize>(ids[j])-1)/rows;
        std::vector<Point> q;
        for (mwSignedIndex c=std::max<mwSignedIndex>(0,col-reachX);c<=std::min<mwSignedIndex>(cols-1,col+reachX);++c)
            for (mwSignedIndex r=std::max<mwSignedIndex>(0,row-reachY);r<=std::min<mwSignedIndex>(rows-1,row+reachY);++r) {
                const mwSignedIndex neighbor=lookup[r+c*rows];if (neighbor<0) continue;
                for (auto i:members[neighbor]) q.push_back({xyz[i],xyz[i+N],xyz[i+2*N],static_cast<mwSize>(neighbor)==j});
            }
        std::vector<Point> seeds{own.front()};std::vector<double> nearest(own.size(),std::numeric_limits<double>::infinity());
        for (int k=1;k<static_cast<int>(p[0]);++k) {
            const auto& last=seeds.back();double far=-1;mwSize winner=0;
            for (mwSize i=0;i<own.size();++i) {
                const double dx=own[i].x-last.x,dy=own[i].y-last.y;nearest[i]=std::min(nearest[i],dx*dx+dy*dy);
                if (nearest[i]>far) {far=nearest[i];winner=i;}
            }
            if (far<p[1]*p[1]) break;seeds.push_back(own[winner]);
        }
        for (const auto& seed:seeds) {
            std::vector<Point> initial;double lo=std::numeric_limits<double>::infinity(),hi=-lo;
            for (const auto& a:own) if (std::hypot(a.x-seed.x,a.y-seed.y)<=p[2]) {initial.push_back(a);lo=std::min(lo,a.z);hi=std::max(hi,a.z);}
            if (initial.size()<p[7] || hi-lo<p[8]) continue;
            std::vector<std::pair<double,double>> intervals;
            for (int w=0;w<=static_cast<int>(p[4]);++w) {
                double low=lo,high=hi;
                if (w>0) {low=lo+(std::max(lo,hi-p[3])-lo)*(p[4]==1?1:static_cast<double>(w-1)/(p[4]-1));high=std::min(low+p[3],hi);}
                const auto interval=std::make_pair(low,high);
                if (std::find(intervals.begin(),intervals.end(),interval)!=intervals.end()) continue;
                intervals.push_back(interval);
                std::vector<Point> fitting;double fitLo=std::numeric_limits<double>::infinity(),fitHi=-fitLo;
                for (const auto& a:initial) if (a.z>=low && a.z<=high) {fitting.push_back(a);fitLo=std::min(fitLo,a.z);fitHi=std::max(fitHi,a.z);}
                if (fitting.size()<p[7] || fitHi-fitLo<p[8]) continue;
                Axis axis=balancedAxis(fitting);if (std::hypot(axis.sx,axis.sy)>maxSlope) continue;
                for (int iteration=0;iteration<2;++iteration) {
                    fitting.clear();for (const auto& a:q) if (a.z>=low && a.z<=high && distance(a,axis)<=p[2]) fitting.push_back(a);
                    if (fitting.size()<p[6]) break;
                    Axis next=balancedAxis(fitting);if (std::hypot(next.sx,next.sy)>maxSlope) break;axis=next;
                }
                std::vector<double> distances(q.size());for (mwSize i=0;i<q.size();++i) distances[i]=distance(q[i],axis);
                for (mwSize radiusIndex=0;radiusIndex<R;++radiusIndex) {
                    const double radius=radii[radiusIndex];std::vector<mwSize> core;std::vector<double> outer;
                    for (mwSize i=0;i<q.size();++i) {if (distances[i]<=radius) core.push_back(i);if (distances[i]<=2*radius) outer.push_back(q[i].z);}
                    if (core.size()<p[6]) continue;
                    std::stable_sort(core.begin(),core.end(),[&](mwSize a,mwSize b){return q[a].z<q[b].z;});std::sort(outer.begin(),outer.end());
                    std::vector<double> z;for (auto i:core) z.push_back(q[i].z);
                    std::vector<mwSize> qualified;std::vector<double> contrast;
                    for (mwSize i=0;i<core.size();++i) {
                        const double inner=windowCount(z,z[i],p[14]),surround=windowCount(outer,z[i],p[14]);
                        const double density=3*inner/std::max(surround-inner,1.0);
                        if (inner>=p[15] && density>=p[16]) {qualified.push_back(core[i]);contrast.push_back(density);}
                    }
                    if (qualified.size()<p[6]) continue;
                    const double gapLimit=p[12]+p[13]*std::hypot(axis.x,axis.y);
                    mwSize start=0;
                    while (start<qualified.size()) {
                        mwSize end=start+1;while (end<qualified.size() && q[qualified[end]].z-q[qualified[end-1]].z<=gapLimit) ++end;
                        if (end-start>=p[6]) {
                            std::vector<double> zz,ratios;double ownLo=std::numeric_limits<double>::infinity(),ownHi=-ownLo;mwSize owned=0;double square=0,gap=0;
                            for (mwSize k=start;k<end;++k) {
                                const auto i=qualified[k];const auto& a=q[i];zz.push_back(a.z);ratios.push_back(contrast[k]);square+=distances[i]*distances[i];
                                if (k>start) gap=std::max(gap,a.z-q[qualified[k-1]].z);
                                if (a.owned) {++owned;ownLo=std::min(ownLo,a.z);ownHi=std::max(ownHi,a.z);}
                            }
                            const double height=zz.back()-zz.front(),ownHeight=ownHi-ownLo,robust=quantile(zz,.95)-quantile(zz,.05),radial=std::sqrt(square/zz.size());
                            if (owned>=p[7] && height>=p[9] && robust>=p[10] && ownHeight>=p[8] && radial<=p[18]) {
                                std::sort(ratios.begin(),ratios.end());const double local=quantile(ratios,.5),peak=peakContrast(q,axis,radius,zz.front(),zz.back());
                                if (peak>=p[17]) {
                                    const double score=std::min(1.0,robust/p[19])*std::min(1.0,zz.size()/p[20])*(1-gap/height)*std::exp(-std::pow(radial/p[18],2))*std::min(1.0,local/p[21])*std::min(1.0,peak/2);
                                    if (score>output[j]+1e-12) {
                                        const double values[18]={score,static_cast<double>(zz.size()),static_cast<double>(owned),height,robust,radial,gap,local,radius,zz.front(),zz.back(),axis.z,ownHeight,peak,axis.x,axis.y,axis.sx,axis.sy};
                                        for (int c=0;c<18;++c) output[j+c*P]=values[c];
                                    }
                                }
                            }
                        }
                        start=end;
                    }
                }
            }
        }
    }
}
} // namespace pole_subset

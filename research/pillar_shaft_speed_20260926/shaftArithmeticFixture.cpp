// Appended after historical and current arithmetic by verifyShaftArithmetic.py.
// These explicit fixture parameters freeze the pre-optimization shaft search.
// This exercises arithmetic equivalence, not accuracy against labeled poles.
bool equal(double a,double b) {
    return a==b || (std::isnan(a) && std::isnan(b));
}

int main(int argc,char** argv) {
    if (argc!=3) return 2;
    const std::uint64_t seed=std::stoull(argv[1]);
    const std::size_t scenes=std::stoull(argv[2]);
    std::mt19937_64 generator(seed);
    std::uniform_real_distribution<double> random(-1,1);
    const double p[29]={4,.08,2,2,.75,10,4,.35,1.3,1,20,.5,.005,.10,
                       1.2,3,12,.30,2,20,.025,.015,1,.65,1,2.5,.65,3,1.05};
    pillar_shaft::FitScratch fit;
    pillar_shaft::IntervalScratch scratch;
    std::size_t fits=0,intervals=0,accepted=0;
    for (std::size_t trial=0;trial<scenes;++trial) {
        std::vector<pole_subset::Point> q;
        const std::size_t n=2+trial%399;
        for (std::size_t i=0;i<n;++i) {
            double z=2*random(generator);
            if (trial%3==0) z=std::round(z*10)/10;
            const double scale=i%5==0?.25:.03;
            q.push_back({scale*random(generator),scale*random(generator),z,i%3!=0});
        }
        if (trial%7==0) for (auto& point:q) point.z=.1;
        for (int vertical=0;vertical<2;++vertical) {
            const auto a=old_shaft::fitAxis(q,vertical);
            const auto b=pillar_shaft::fitAxis(q,vertical,fit);
            const double aa[]={a.x,a.y,a.z,a.sx,a.sy},bb[]={b.x,b.y,b.z,b.sx,b.sy};
            for (int k=0;k<5;++k) if (!equal(aa[k],bb[k])) {
                std::cerr<<"Fit mismatch: scene="<<trial<<", vertical="<<vertical<<", field="<<k<<"\n";
                return 1;
            }
            ++fits;
        }
        std::stable_sort(q.begin(),q.end(),[](const auto& a,const auto& b){return a.z<b.z;});
        const auto axis=old_shaft::fitAxis(q,trial%2);
        std::vector<pillar_shaft::AxisSample> samples;
        for (const auto& point:q) {
            const double x=point.x-axis.x-(point.z-axis.z)*axis.sx;
            const double y=point.y-axis.y-(point.z-axis.z)*axis.sy;
            const double d=std::hypot(x,y);
            if (d<=.3*(1+1e-14)) samples.push_back({point,x,y,d});
        }
        double before[23]={},after[23]={};
        before[14]=before[15]=after[14]=after[15]=std::numeric_limits<double>::quiet_NaN();
        for (double radius:{.15,.06,.10}) {
            old_shaft::intervals(q,axis,radius,p,before,0,1);
            pillar_shaft::intervals(samples,axis,radius,p,after,0,1,scratch);
            for (int k=0;k<23;++k) if (!equal(before[k],after[k])) {
                std::cerr<<"Interval mismatch: scene="<<trial<<", radius="<<radius<<", field="<<k<<"\n";
                return 1;
            }
            ++intervals;
        }
        accepted+=after[0]>0;
    }
    std::cout<<"{\"passed\":true,\"seed\":"<<seed<<",\"scenes\":"<<scenes
             <<",\"exact_axis_fits\":"<<fits<<",\"exact_interval_evaluations\":"<<intervals
             <<",\"accepted_scenes\":"<<accepted<<"}\n";
}

// Exact CPU kernels for stateful raster traversals. No MATLAB input is
// mutated, no frame state is retained, and no fast-math option is required.
#include "mex.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>
#include <stdexcept>
#include <limits>
#include <vector>

namespace {
void require(bool condition, const char* message) {
    if (!condition) mexErrMsgIdAndTxt("perception:native:InvalidInput", "%s", message);
}
void array(const mxArray* a, mxClassID type, mwSize count) {
    require(mxGetClassID(a) == type && !mxIsComplex(a) && !mxIsSparse(a)
        && mxGetNumberOfElements(a) == count, "Unexpected array type or size.");
}
bool index(double x, mwSize count) {
    return std::isfinite(x) && x >= 1 && x <= static_cast<double>(count) && x == std::floor(x);
}
double smallMedian(double* v, mwSize n) {
    std::sort(v, v+n);
    return n % 2 ? v[n/2] : 0.5 * (v[n/2-1] + v[n/2]);
}
void propagate(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs == 11 && nlhs == 2, "Propagation needs ten inputs and two outputs.");
    const mwSize n = mxGetNumberOfElements(in[1]);
    const mwSize cells = mxGetNumberOfElements(in[6]);
    array(in[1], mxDOUBLE_CLASS, n); // Stable near-to-far order, one based.
    array(in[2], mxDOUBLE_CLASS, n); // Occupied raster indices.
    array(in[3], mxDOUBLE_CLASS, n); // Robust low heights.
    array(in[4], mxLOGICAL_CLASS, n);
    array(in[5], mxDOUBLE_CLASS, n); // Ranges.
    array(in[6], mxDOUBLE_CLASS, cells);
    array(in[7], mxUINT8_CLASS, cells);
    array(in[8], mxDOUBLE_CLASS, 2);
    array(in[9], mxDOUBLE_CLASS, 2);
    array(in[10], mxDOUBLE_CLASS, 5);
    const double* order = mxGetDoubles(in[1]);
    const double* occupied = mxGetDoubles(in[2]);
    const double* low = mxGetDoubles(in[3]);
    const mxLogical* flat = mxGetLogicals(in[4]);
    const double* range = mxGetDoubles(in[5]);
    const double* dims = mxGetDoubles(in[8]);
    const double* spacing = mxGetDoubles(in[9]);
    const double* p = mxGetDoubles(in[10]);
    require(index(dims[0], cells) && index(dims[1], cells)
        && dims[0]*dims[1] == static_cast<double>(cells), "Invalid raster dimensions.");
    for (mwSize k=0; k<n; ++k) {
        require(index(order[k], n) && index(occupied[k], cells)
            && std::isfinite(low[k]) && std::isfinite(range[k]), "Invalid cell statistics.");
    }
    for (int k=0; k<2; ++k) require(std::isfinite(spacing[k]) && spacing[k]>0, "Invalid cell spacing.");
    for (int k=0; k<5; ++k) require(std::isfinite(p[k]), "Invalid propagation parameter.");
    out[0] = mxDuplicateArray(in[6]);
    out[1] = mxDuplicateArray(in[7]);
    double* height = mxGetDoubles(out[0]);
    mxUint8* state = mxGetUint8s(out[1]);
    const mwSignedIndex nx = static_cast<mwSignedIndex>(dims[0]);
    const mwSignedIndex ny = static_cast<mwSignedIndex>(dims[1]);
    const int dx[8] = {-1,0,1,-1,1,-1,0,1};
    const int dy[8] = {-1,-1,-1,0,0,1,1,1};
    double distance[8];
    for (int k=0; k<8; ++k) distance[k] = std::hypot(dx[k]*spacing[0], dy[k]*spacing[1]);
    for (mwSize visit=0; visit<n; ++visit) {
        const mwSize s = static_cast<mwSize>(order[visit])-1;
        const mwSize cell = static_cast<mwSize>(occupied[s])-1;
        if (state[cell] == 1) continue;
        const mwSignedIndex x = cell % nx, y = cell / nx;
        const double noise = std::min(p[0] + p[1]*range[s], p[2]);
        double minimumResidual=std::numeric_limits<double>::infinity();
        double minimumAllowed=std::numeric_limits<double>::infinity();
        double neighbors[8];
        mwSize count = 0;
        for (int k=0; k<8; ++k) {
            const mwSignedIndex xx=x+dx[k], yy=y+dy[k];
            if (xx<0 || xx>=nx || yy<0 || yy>=ny) continue;
            const mwSize neighbor = xx+yy*nx;
            if (state[neighbor] != 1 || !std::isfinite(height[neighbor])) continue;
            neighbors[count++] = height[neighbor];
            minimumResidual=std::min(minimumResidual,std::abs(low[s]-height[neighbor]));
            minimumAllowed=std::min(minimumAllowed,p[3]+p[4]*distance[k]+noise);
        }
        if (!count) continue;
        const bool compatible=flat[s] && minimumResidual<minimumAllowed;
        height[cell] = compatible ? low[s] : smallMedian(neighbors, count);
        state[cell] = compatible ? 1 : 2;
    }
}
void roadGrowth(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs == 5 && nlhs == 1, "Road growth needs four inputs and one output.");
    const mwSize rows=mxGetM(in[1]), cols=mxGetN(in[1]), n=rows*cols;
    array(in[1], mxLOGICAL_CLASS, n);
    array(in[2], mxDOUBLE_CLASS, mxGetNumberOfElements(in[2])); // Seed indices.
    array(in[3], mxDOUBLE_CLASS, n);
    array(in[4], mxDOUBLE_CLASS, 3); // Seed height, neighbor step, seed deviation.
    require(mxGetNumberOfDimensions(in[1]) == 2, "Road mask must be a matrix.");
    const mxLogical* candidate=mxGetLogicals(in[1]);
    const double* seeds=mxGetDoubles(in[2]);
    const double* height=mxGetDoubles(in[3]);
    const double* p=mxGetDoubles(in[4]);
    const mwSize numSeeds=mxGetNumberOfElements(in[2]);
    for (mwSize k=0; k<numSeeds; ++k) require(index(seeds[k], n), "Invalid seed index.");
    out[0]=mxCreateLogicalMatrix(rows, cols);
    mxLogical* reached=mxGetLogicals(out[0]);
    std::vector<mwSize> queue;
    queue.reserve(n);
    for (mwSize k=0; k<numSeeds; ++k) {
        const mwSize s=static_cast<mwSize>(seeds[k])-1;
        if (candidate[s] && std::isfinite(height[s]) && !reached[s]) {
            queue.push_back(s); reached[s]=true;
        }
    }
    for (mwSize head=0; head<queue.size(); ++head) {
        const mwSize cell=queue[head];
        const mwSignedIndex row=cell%rows, col=cell/rows;
        for (int dr=-1; dr<=1; ++dr) {
            const mwSignedIndex rr=row+dr;
            if (rr<0 || rr>=static_cast<mwSignedIndex>(rows)) continue;
            for (int dc=-1; dc<=1; ++dc) {
                const mwSignedIndex cc=col+dc;
                if ((!dr && !dc) || cc<0 || cc>=static_cast<mwSignedIndex>(cols)) continue;
                const mwSize next=rr+cc*rows;
                if (reached[next] || !candidate[next] || !std::isfinite(height[next])) continue;
                if (std::isfinite(p[1]) && std::abs(height[next]-height[cell]) > p[1]) continue;
                if (std::isfinite(p[2]) && std::abs(height[next]-p[0]) > p[2]) continue;
                queue.push_back(next); reached[next]=true;
            }
        }
    }
}
double smooth(double x, double lower, double upper) {
    if (upper<=lower) return x>=upper ? 1 : 0;
    const double t=std::min(std::max((x-lower)/(upper-lower), 0.0), 1.0);
    return 3*t*t-2*t*t*t;
}
void groundStats(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs==4 && nlhs==1, "Ground statistics need three inputs and one output.");
    const mwSize n=mxGetNumberOfElements(in[1]);
    array(in[1],mxDOUBLE_CLASS,n); array(in[2],mxDOUBLE_CLASS,n); array(in[3],mxDOUBLE_CLASS,1);
    const double size=mxGetScalar(in[3]);
    require(size>=1 && size<=2147483647 && size==std::floor(size),"Invalid grid size.");
    const mwSize cells=static_cast<mwSize>(size);
    const double* indices=mxGetDoubles(in[1]); const double* z=mxGetDoubles(in[2]);
    for (mwSize k=0;k<n;++k) require(index(indices[k],cells) && std::isfinite(z[k]),"Invalid ground sample.");
    const double inf=std::numeric_limits<double>::infinity();
    std::vector<mwSize> counts(cells,0);
    std::vector<double> low(cells,inf),second(cells,inf),high(cells,-inf);
    mwSize occupied=0;
    for (mwSize k=0;k<n;++k) {
        const mwSize cell=static_cast<mwSize>(indices[k])-1;
        if (!counts[cell]++) ++occupied;
        if (z[k]<low[cell]) {second[cell]=low[cell];low[cell]=z[k];}
        else if (z[k]<second[cell]) second[cell]=z[k];
        high[cell]=std::max(high[cell],z[k]);
    }
    out[0]=mxCreateDoubleMatrix(occupied,5,mxREAL);
    double* result=mxGetDoubles(out[0]); mwSize row=0;
    for (mwSize cell=0;cell<cells;++cell) {
        if (!counts[cell]) continue;
        result[row]=cell+1; result[row+occupied]=counts[cell]; result[row+2*occupied]=low[cell];
        result[row+3*occupied]=counts[cell]==1 ? low[cell] : second[cell];
        result[row+4*occupied]=high[cell]; ++row;
    }
}
mwSize findRoot(std::vector<mwSize>& parent,mwSize node) {
    while (parent[node]!=node) {parent[node]=parent[parent[node]];node=parent[node];}
    return node;
}
void smoothComponents(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs==7 && nlhs==1,"Smooth components need six inputs and one output.");
    const mwSize n=mxGetNumberOfElements(in[1]);
    array(in[1],mxDOUBLE_CLASS,n); array(in[2],mxDOUBLE_CLASS,n); array(in[3],mxDOUBLE_CLASS,n);
    array(in[4],mxDOUBLE_CLASS,2); array(in[5],mxDOUBLE_CLASS,2); array(in[6],mxDOUBLE_CLASS,5);
    const double* cells=mxGetDoubles(in[1]); const double* low=mxGetDoubles(in[2]);
    const double* range=mxGetDoubles(in[3]); const double* dims=mxGetDoubles(in[4]);
    const double* spacing=mxGetDoubles(in[5]); const double* p=mxGetDoubles(in[6]);
    require(index(dims[0],2147483647) && index(dims[1],2147483647)
        && dims[0]*dims[1]<=2147483647,"Invalid component grid dimensions.");
    const mwSize total=static_cast<mwSize>(dims[0]*dims[1]);
    for (mwSize k=0;k<n;++k) require(index(cells[k],total)&&std::isfinite(low[k])&&std::isfinite(range[k]),"Invalid component sample.");
    for (int k=0;k<2;++k) require(std::isfinite(spacing[k])&&spacing[k]>0,"Invalid component spacing.");
    for (int k=0;k<5;++k) require(std::isfinite(p[k]),"Invalid component parameter.");
    std::vector<mwSize> lookup(total,n),parent(n),rank(n,0),labels(n,0);
    std::vector<double> noise(n);
    for (mwSize k=0;k<n;++k) {
        parent[k]=k;lookup[static_cast<mwSize>(cells[k])-1]=k;
        noise[k]=std::min(p[0]+p[1]*range[k],p[2]);
    }
    const mwSignedIndex nx=static_cast<mwSignedIndex>(dims[0]),ny=static_cast<mwSignedIndex>(dims[1]);
    const int dx[4]={1,0,1,1},dy[4]={0,1,1,-1};
    for (int direction=0;direction<4;++direction) {
        const double base=p[3]+p[4]*std::hypot(dx[direction]*spacing[0],dy[direction]*spacing[1]);
        for (mwSize k=0;k<n;++k) {
            const mwSize cell=static_cast<mwSize>(cells[k])-1;
            const mwSignedIndex x=static_cast<mwSignedIndex>(cell%nx)+dx[direction];
            const mwSignedIndex y=static_cast<mwSignedIndex>(cell/nx)+dy[direction];
            if (x<0||x>=nx||y<0||y>=ny) continue;
            const mwSize other=lookup[x+y*nx];
            if (other==n || std::abs(low[k]-low[other])>=base+std::max(noise[k],noise[other])) continue;
            mwSize a=findRoot(parent,k),b=findRoot(parent,other);
            if (a==b) continue;
            if (rank[a]<rank[b]) std::swap(a,b);
            parent[b]=a;if (rank[a]==rank[b]) ++rank[a];
        }
    }
    out[0]=mxCreateDoubleMatrix(n,1,mxREAL);double* result=mxGetDoubles(out[0]);mwSize label=0;
    // Match MATLAB conncomp numbering: components ordered by first member.
    for (mwSize k=0;k<n;++k) {
        const mwSize root=findRoot(parent,k);
        if (!labels[root]) labels[root]=++label;
        result[k]=labels[root];
    }
}
void cellMoments(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs==4 && nlhs==3,"Cell moments need points, indices, size and three outputs.");
    const mwSize n=mxGetM(in[1]), dimensions=mxGetN(in[1]);
    require(mxGetNumberOfDimensions(in[1])==2 && (dimensions==2 || dimensions==3),"Expected XY or XYZ points.");
    array(in[1],mxDOUBLE_CLASS,n*dimensions); array(in[2],mxDOUBLE_CLASS,n); array(in[3],mxDOUBLE_CLASS,1);
    const double size=mxGetScalar(in[3]);
    require(std::isfinite(size) && size>=0 && size<=2147483647 && size==std::floor(size),"Invalid cell count.");
    const mwSize cells=static_cast<mwSize>(size);
    const double* points=mxGetDoubles(in[1]); const double* indices=mxGetDoubles(in[2]);
    for (mwSize k=0;k<n;++k) require(index(indices[k],cells),"Invalid moment index.");
    out[0]=mxCreateDoubleMatrix(cells,1,mxREAL);
    out[1]=mxCreateDoubleMatrix(cells,dimensions,mxREAL);
    out[2]=mxCreateDoubleMatrix(cells,dimensions==3 ? 6 : 3,mxREAL);
    double* count=mxGetDoubles(out[0]); double* mean=mxGetDoubles(out[1]); double* covariance=mxGetDoubles(out[2]);
    // Two centered passes retain small scatter at large coordinate offsets.
    // Each cell's additions follow the original point order, as in accumarray.
    for (mwSize k=0;k<n;++k) {
        const mwSize c=static_cast<mwSize>(indices[k])-1;
        ++count[c];
        for (mwSize d=0;d<dimensions;++d) mean[c+d*cells]+=points[k+d*n];
    }
    for (mwSize c=0;c<cells;++c) if (count[c]) {
        for (mwSize d=0;d<dimensions;++d) mean[c+d*cells]/=count[c];
    }
    for (mwSize k=0;k<n;++k) {
        const mwSize c=static_cast<mwSize>(indices[k])-1;
        const double x=points[k]-mean[c],y=points[k+n]-mean[c+cells];
        covariance[c]+=x*x; covariance[c+cells]+=x*y; covariance[c+2*cells]+=y*y;
        if (dimensions==3) {
            const double z=points[k+2*n]-mean[c+2*cells];
            covariance[c+3*cells]+=x*z; covariance[c+4*cells]+=y*z; covariance[c+5*cells]+=z*z;
        }
    }
    for (mwSize c=0;c<cells;++c) if (count[c]) {
        for (mwSize d=0;d<(dimensions==3 ? 6 : 3);++d) covariance[c+d*cells]/=count[c];
    }
}
void neighborDifference(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs==3 && nlhs==1,"Neighbor difference needs a height map and validity mask.");
    const mwSize rows=mxGetM(in[1]),cols=mxGetN(in[1]),n=rows*cols;
    require(mxGetNumberOfDimensions(in[1])==2,"Expected a height matrix.");
    array(in[1],mxDOUBLE_CLASS,n); array(in[2],mxLOGICAL_CLASS,n);
    const double* values=mxGetDoubles(in[1]); const mxLogical* valid=mxGetLogicals(in[2]);
    out[0]=mxCreateDoubleMatrix(rows,cols,mxREAL); double* result=mxGetDoubles(out[0]);
    for (mwSize c=0;c<n;++c) {
        if (!valid[c] || !std::isfinite(values[c])) continue;
        const mwSignedIndex row=c%rows,col=c/rows;
        double largest=0;
        for (int dc=-1;dc<=1;++dc) for (int dr=-1;dr<=1;++dr) {
            const mwSignedIndex rr=row+dr,cc=col+dc;
            if ((!dr&&!dc) || rr<0 || cc<0 || rr>=static_cast<mwSignedIndex>(rows) || cc>=static_cast<mwSignedIndex>(cols)) continue;
            const mwSize other=rr+cc*rows;
            if (valid[other] && std::isfinite(values[other])) largest=std::max(largest,std::abs(values[c]-values[other]));
        }
        result[c]=largest;
    }
}
void directionalSupport(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs==4 && nlhs==1,"Directional support needs seed/active masks and parameters.");
    const mwSize rows=mxGetM(in[1]),cols=mxGetN(in[1]),n=rows*cols;
    require(mxGetNumberOfDimensions(in[1])==2,"Expected a seed matrix.");
    array(in[1],mxLOGICAL_CLASS,n); array(in[2],mxLOGICAL_CLASS,n); array(in[3],mxDOUBLE_CLASS,5);
    const mxLogical* seed=mxGetLogicals(in[1]); const mxLogical* active=mxGetLogicals(in[2]);
    const double* p=mxGetDoubles(in[3]);
    for (int k=0;k<5;++k) require(std::isfinite(p[k]),"Invalid directional parameter.");
    require(p[0]>=1 && p[0]<=10000 && p[0]==std::floor(p[0]),"Invalid directional radius.");
    const int radius=static_cast<int>(p[0]);
    const int dr[4]={0,1,1,1},dc[4]={1,0,1,-1};
    out[0]=mxCreateDoubleMatrix(rows,cols,mxREAL); double* result=mxGetDoubles(out[0]);
    for (mwSize c=0;c<n;++c) {
        if (!active[c]) continue;
        const mwSignedIndex row=c%rows,col=c/rows;
        for (int direction=0;direction<4;++direction) {
            const int rr=dr[direction],cc=dc[direction],nr=-cc,nc=rr;
            const int rowHalo=radius*std::abs(rr)+std::abs(nr),colHalo=radius*std::abs(cc)+std::abs(nc);
            if (row<rowHalo || col<colHalo || row+rowHalo>=static_cast<mwSignedIndex>(rows) || col+colHalo>=static_cast<mwSignedIndex>(cols)) continue;
            double line=0,positive=0,negative=0;
            for (int step=-radius;step<=radius;++step) {
                const mwSignedIndex r=row+step*rr,k=col+step*cc;
                line+=seed[r+k*rows]; positive+=seed[r+nr+(k+nc)*rows]; negative+=seed[r-nr+(k-nc)*rows];
            }
            const double thinness=line/std::max(line+std::min(positive,negative),std::numeric_limits<double>::epsilon());
            result[c]=std::max(result[c],smooth(line,p[1],p[2])*smooth(thinness,p[3],p[4]));
        }
    }
}
void pillarShape(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs==4 && nlhs==3, "Pillar shape needs three inputs and three outputs.");
    const mwSize rows=mxGetM(in[1]), cols=mxGetN(in[1]), n=rows*cols;
    array(in[1], mxSINGLE_CLASS, n);
    array(in[2], mxLOGICAL_CLASS, n);
    array(in[3], mxDOUBLE_CLASS, 7);
    require(mxGetNumberOfDimensions(in[1])==2, "Run map must be a matrix.");
    const float* run=mxGetSingles(in[1]);
    const mxLogical* occupied=mxGetLogicals(in[2]);
    const double* p=mxGetDoubles(in[3]);
    for (int k=0; k<7; ++k) require(std::isfinite(p[k]), "Invalid shape parameter.");
    require(p[0]>=1 && p[0]<=10000 && p[0]==std::floor(p[0]) && p[1]>0 && p[2]>0,
        "Invalid neighborhood radius or spacing.");
    const int radius=static_cast<int>(p[0]);
    const double eps=std::numeric_limits<double>::epsilon();
    std::vector<double> weights(n);
    for (mwSize i=0; i<n; ++i) {
        weights[i]=occupied[i] && std::isfinite(run[i])
            ? std::pow(std::max(static_cast<double>(run[i]), 0.0), p[3]) : 0;
        if (!std::isfinite(weights[i])) weights[i]=0;
    }
    for (int k=0; k<3; ++k) out[k]=mxCreateNumericMatrix(rows, cols, mxSINGLE_CLASS, mxREAL);
    float* point=mxGetSingles(out[0]);
    float* line=mxGetSingles(out[1]);
    float* orientation=mxGetSingles(out[2]);
    std::fill(orientation, orientation+n, std::numeric_limits<float>::quiet_NaN());
    const int dr[4]={0,1,1,1}, dc[4]={1,0,-1,1};
    for (mwSize i=0; i<n; ++i) {
        if (!occupied[i]) continue;
        const mwSignedIndex row=i%rows, col=i/rows;
        float drops[4]={0,0,0,0};
        for (int direction=0; direction<4; ++direction) {
            for (int step=1; step<=radius; ++step) {
                const mwSignedIndex ra=std::clamp(row+step*dr[direction], mwSignedIndex(0), mwSignedIndex(rows)-1);
                const mwSignedIndex rb=std::clamp(row-step*dr[direction], mwSignedIndex(0), mwSignedIndex(rows)-1);
                const mwSignedIndex ca=std::clamp(col+step*dc[direction], mwSignedIndex(0), mwSignedIndex(cols)-1);
                const mwSignedIndex cb=std::clamp(col-step*dc[direction], mwSignedIndex(0), mwSignedIndex(cols)-1);
                drops[direction]=std::max(drops[direction], 2*run[i]-(run[ra+ca*rows]+run[rb+cb*rows]));
            }
        }
        const float largest=*std::max_element(drops,drops+4), smallest=*std::min_element(drops,drops+4);
        if (largest>std::numeric_limits<float>::epsilon())
            point[i]=smallest/(largest+std::numeric_limits<float>::epsilon());
        double m00=0,m10=0,m01=0,m20=0,m02=0,m11=0,seeds=0;
        for (int cc=-radius; cc<=radius; ++cc) {
            if (col+cc<0 || col+cc>=static_cast<mwSignedIndex>(cols)) continue;
            for (int rr=-radius; rr<=radius; ++rr) {
                if (row+rr<0 || row+rr>=static_cast<mwSignedIndex>(rows)) continue;
                const double w=weights[row+rr+(col+cc)*rows], x=cc*p[1], y=rr*p[2];
                m00+=w; m10+=w*x; m01+=w*y; m20+=w*x*x; m02+=w*y*y; m11+=w*x*y;
                seeds+=w>0;
            }
        }
        if (m00<=eps || seeds<p[5]) continue;
        const double inverse=1/std::max(m00,eps), x=m10*inverse, y=m01*inverse;
        const double xx=std::max(m20*inverse-x*x,0.0), yy=std::max(m02*inverse-y*y,0.0), xy=m11*inverse-x*y;
        const double trace=xx+yy, delta=std::sqrt((xx-yy)*(xx-yy)+4*xy*xy);
        const double l1=std::max(0.5*(trace+delta),0.0), l2=std::max(std::min(0.5*(trace-delta),l1),0.0);
        const double anisotropy=std::clamp((l1-l2)/std::max(l1+l2,eps),0.0,1.0);
        const double score=std::pow(anisotropy,p[4])*smooth(seeds,p[5],p[6]);
        line[i]=static_cast<float>(score);
        if (score>std::numeric_limits<float>::epsilon()) {
            const double angle=0.5*std::atan2(2*xy,xx-yy)*(180/std::acos(-1.0));
            orientation[i]=static_cast<float>(std::fmod(angle+180,180)-90);
        }
    }
}
}
void mexFunction(int nlhs, mxArray** out, int nrhs, const mxArray** in) {
    require(nrhs>0 && mxIsChar(in[0]), "First input must name a kernel.");
    char command[48];
    require(mxGetString(in[0], command, sizeof(command)) == 0, "Invalid kernel name.");
    try {
        if (!std::strcmp(command,"version")) {
            require(nrhs==1 && nlhs==1,"Version needs no data and one output.");
            out[0]=mxCreateDoubleScalar(2);
        }
        else if (!std::strcmp(command, "propagateGround")) propagate(nlhs, out, nrhs, in);
        else if (!std::strcmp(command, "growRoad")) roadGrowth(nlhs, out, nrhs, in);
        else if (!std::strcmp(command, "pillarShape")) pillarShape(nlhs, out, nrhs, in);
        else if (!std::strcmp(command, "cellMoments")) cellMoments(nlhs, out, nrhs, in);
        else if (!std::strcmp(command, "neighborDifference")) neighborDifference(nlhs, out, nrhs, in);
        else if (!std::strcmp(command, "directionalSupport")) directionalSupport(nlhs, out, nrhs, in);
        else if (!std::strcmp(command, "groundStats")) groundStats(nlhs, out, nrhs, in);
        else if (!std::strcmp(command, "smoothComponents")) smoothComponents(nlhs, out, nrhs, in);
        else mexErrMsgIdAndTxt("perception:native:UnknownKernel", "Unknown kernel.");
    } catch (const std::bad_alloc& error) {
        mexErrMsgIdAndTxt("perception:native:AllocationFailure","%s",error.what());
    } catch (const std::length_error& error) {
        mexErrMsgIdAndTxt("perception:native:AllocationFailure","%s",error.what());
    }
}

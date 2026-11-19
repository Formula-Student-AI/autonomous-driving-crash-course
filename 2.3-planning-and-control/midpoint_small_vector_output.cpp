// FSAI cone-map centreline planner and lap tracker. C++17, standard library only.
// Free-form figure eight: c++ -std=c++17 -O2 midpoint_small_vector_output.cpp -o planner
// ./planner --figure8 0.60 < figure8_original.txt > route.json
// The argument is a minimum distance (m) from each OUTPUT segment to every cone
// centre. Select it from your geometry. Vehicle body, steering and dynamics are
// outside this model. The default mode retains the ordinary course planner.
// Design reference: BCN eMotorsport Urinay (https://github.com/origovi/urinay).
// The paired-gate circuit search and lateral-offset implementation here are original.
// Run: ./fsai_centerline < map.txt
// Map input (whitespace-separated): blue_count yellow_count closed_flag spacing_m,
// car_x car_y heading_x heading_y, then blue coordinates, then yellow coordinates.
// Example: echo '3 3 0 0.2 0 0 1 0 0 2 3 2 6 2 0 -2 3 -2 6 -2' | ./fsai_centerline
// JSON vectors are successive (dx, dy) in map metres, relative to start_position.
// --track reads point_count target_laps update_count, then closed route points,
// then one x y heading_x heading_y dt_seconds record per update.

#include <string>
#include <vector>

namespace fsai {
struct Vec2 {
    double x=0, y=0;
    Vec2 operator+(Vec2 b) const { return {x+b.x,y+b.y}; }
    Vec2 operator-(Vec2 b) const { return {x-b.x,y-b.y}; }
    Vec2 operator*(double s) const { return {x*s,y*s}; }
    Vec2 operator/(double s) const { return {x/s,y/s}; }
};
double dot(Vec2 a, Vec2 b);
double norm(Vec2 a);
struct Options {
    bool closed=false;
    double figure_clearance=0;
    double spacing=0.2, dense_step=0.01;
    double min_width=2, max_width=7, max_start_distance=8, max_turn_deg=80;
    int lookahead=5, beam_width=8;
};
struct Path {
    std::string frame="map", status;
    bool closed=false;
    int bridge_count=0;
    double requested_spacing=0.2, actual_arc_spacing=0, arc_length=0, omitted_tail=0;
    std::vector<Vec2> midpoints, dense_points, points, vectors;
    std::vector<double> signed_curvature;
};
// Blue must be left of the intended travel direction. Throws on unavailable route.
// Standalone experimental port: lookahead + interpolating spline, no smoothing.
Path plan(const std::vector<Vec2>& blue, const std::vector<Vec2>& yellow,
          Vec2 car, Vec2 heading, const Options& options={});
struct Progress {
    double distance=0, lap_progress=0, tracking_error=0;
    int completed_laps=0;
    bool finished=false;
};
class RouteProgress {
 public:
    explicit RouteProgress(const std::vector<Vec2>& points, int target_laps=10,
                           double max_speed=15, double max_error=0.5, double search_window=5);
    Progress update(Vec2 position, Vec2 heading, double dt);
    Progress state() const;
    double length() const { return arc_.back(); }
 private:
    std::vector<Vec2> points_,delta_;
    std::vector<double> arc_,lengths_;
    int target_;
    double max_speed_, max_error_, window_, distance_=0, error_=0;
    Vec2 last_position_;
    bool has_position_=false;
};
}

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace fsai {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
double norm(Vec2 a) { return std::hypot(a.x,a.y); }
namespace {
constexpr double pi=3.14159265358979323846;
bool finite(Vec2 a) { return std::isfinite(a.x)&&std::isfinite(a.y); }
void require(bool ok,const char* message) { if(!ok) throw std::invalid_argument(message); }
using Edge=std::pair<int,int>;
using Triangle=std::array<int,3>;
Edge edge(int a,int b) { return std::minmax(a,b); }
long double orient(Vec2 a,Vec2 b,Vec2 c) {
    return ((long double)b.x-a.x)*((long double)c.y-a.y)-((long double)b.y-a.y)*((long double)c.x-a.x);
}
// Bowyer-Watson on normalized coordinates. Long-double predicates reduce rounding
// error but are NOT exact adaptive predicates. Degenerate real maps need a robust
// geometry backend before deployment; cocircular diagonals may differ from Qhull.
bool in_circle(Vec2 p,Vec2 a,Vec2 b,Vec2 c) {
    long double ax=(long double)a.x-p.x, ay=(long double)a.y-p.y;
    long double bx=(long double)b.x-p.x, by=(long double)b.y-p.y;
    long double cx=(long double)c.x-p.x, cy=(long double)c.y-p.y;
    long double det=(ax*ax+ay*ay)*(bx*cy-by*cx)
                   -(bx*bx+by*by)*(ax*cy-ay*cx)
                   +(cx*cx+cy*cy)*(ax*by-ay*bx);
    return det* (orient(a,b,c)>0?1:-1)>0;
}
std::vector<Triangle> triangulate(const std::vector<Vec2>& input) {
    const int n=(int)input.size();
    double xmin=input[0].x,xmax=xmin,ymin=input[0].y,ymax=ymin;
    for(auto p:input) { xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y); }
    double scale=std::max(xmax-xmin,ymax-ymin);
    require(scale>0&&std::isfinite(scale),"Invalid map scale");
    Vec2 center{xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2};
    std::vector<Vec2> p; for(auto q:input)p.push_back((q-center)/scale);
    p.push_back({-64,-32});p.push_back({64,-32});p.push_back({0,64});
    std::vector<Triangle> tris{{n,n+1,n+2}};
    for(int i=0;i<n;++i) {
        std::map<Edge,int> boundary;std::vector<Triangle> keep;
        for(auto t:tris) {
            if(in_circle(p[i],p[t[0]],p[t[1]],p[t[2]]))
                for(int k=0;k<3;++k)++boundary[edge(t[k],t[(k+1)%3])];
            else keep.push_back(t);
        }
        require(!boundary.empty(),"Triangulation insertion failed");
        for(auto item:boundary) if(item.second==1) {
            auto [a,b]=item.first;auto area=orient(p[a],p[b],p[i]);
            if(area==0)continue;
            if(area<0)std::swap(a,b);
            keep.push_back({a,b,i});
        }
        tris=std::move(keep);
    }
    tris.erase(std::remove_if(tris.begin(),tris.end(),[n](Triangle t){return t[0]>=n||t[1]>=n||t[2]>=n;}),tris.end());
    require(!tris.empty(),"Map is collinear or cannot be triangulated");
    std::set<int> covered; for(auto t:tris)for(int i:t)covered.insert(i);
    require((int)covered.size()==n,"Triangulation omitted a cone");
    return tris;
}
struct Gate { Edge ids; Vec2 mid,tangent;double width;std::set<int> neighbours; };
struct Step { double cost,ds,angle;int gate;Vec2 unit;bool bridge,closing; };
using Seen=std::map<int,std::vector<Vec2>>;
struct SearchState { double cost=0,length=0,advance=0;int gate=0;Vec2 direction;Seen seen;std::vector<Step> sequence; };
struct Route { std::vector<Vec2> mid;int bridges=0;std::string status="no_candidate"; };
Route select(const std::vector<Vec2>& cones,const std::vector<Gate>& gates,Vec2 car,Vec2 heading,const Options& cfg) {
    int start=-1;double best=std::numeric_limits<double>::infinity();
    for(int i=0;i<(int)gates.size();++i) {
        auto d=gates[i].mid-car;double ds=norm(d);
        if(dot(d,heading)>=-1e-8&&ds<=cfg.max_start_distance&&dot(gates[i].tangent,heading)>0&&ds<best) {start=i;best=ds;}
    }
    require(start>=0,"No forward oriented start gate");
    std::map<int,double> minimum_width;
    for(auto& g:gates)for(int i:{g.ids.first,g.ids.second}) {
        auto it=minimum_width.find(i);
        if(it==minimum_width.end()||g.width<it->second)minimum_width[i]=g.width;
    }
    std::vector<double> widths;for(auto w:minimum_width)widths.push_back(w.second);
    std::sort(widths.begin(),widths.end());auto n=widths.size();double width_ref=(widths[(n-1)/2]+widths[n/2])/2;
    auto options=[&](int current,Vec2 direction,const Seen& used,double length) {
        std::vector<Step> rows;
        for(int j=0;j<(int)gates.size();++j) {
            auto delta=gates[j].mid-gates[current].mid;double ds=norm(delta);
            if(ds<=1e-6||ds>10)continue;
            auto unit=delta/ds;double angle=std::acos(std::clamp(dot(direction,unit),-1.,1.));
            if(angle>cfg.max_turn_deg*pi/180)continue;
            bool closing=j==start&&cfg.closed&&length>20&&dot(unit,heading)>.85;
            auto it=used.find(j);bool visited=false;
            if(it!=used.end())for(auto old:it->second)if(dot(unit,old)>.5)visited=true;
            if(visited&&!closing)continue;
            double align=dot(gates[j].tangent,unit);
            if(align<.45)continue;
            bool bridge=!gates[current].neighbours.count(j);
            if(bridge&&(angle>18*pi/180||align<.85||dot(gates[current].tangent,unit)<.85||gates[j].width<.65*width_ref||gates[j].width>1.5*width_ref))continue;
            bool clear=true;
            for(auto c:cones) {
                double s=std::clamp(dot(c-gates[current].mid,unit),0.,ds);
                if(norm(c-(gates[current].mid+unit*s))<.6){clear=false;break;}
            }
            if(!clear)continue;
            double w=(gates[j].width-width_ref)/width_ref;
            double cost=5*angle*angle+.25*w*w+.1*(1-align)*(1-align)+(bridge?.12+.015*ds:0);
            rows.push_back({cost,ds,angle,j,unit,bridge,closing});
        }
        bool ordinary=std::any_of(rows.begin(),rows.end(),[](auto s){return !s.bridge&&s.angle<25*pi/180;});
        if(ordinary)rows.erase(std::remove_if(rows.begin(),rows.end(),[](auto s){return s.bridge;}),rows.end());
        std::sort(rows.begin(),rows.end(),[](auto a,auto b){return std::tie(a.cost,a.ds,a.gate)<std::tie(b.cost,b.ds,b.gate);});
        if(rows.size()>12)rows.resize(12);return rows;
    };
    auto score=[](const SearchState& s){return s.cost-.08*std::min(s.advance,8.);};
    Route route;route.mid.push_back(gates[start].mid);
    SearchState committed;committed.gate=start;committed.direction=heading;committed.seen[start].push_back(heading);
    int budget=4*(int)gates.size()+1;
    for(int step=0;step<budget;++step) {
        if(options(committed.gate,committed.direction,committed.seen,committed.length).empty())return route;
        auto initial=committed;initial.cost=0;initial.advance=0;initial.sequence.clear();
        std::vector<SearchState> beam{initial};
        for(int depth=0;depth<cfg.lookahead;++depth) {
            std::vector<SearchState> children;
            for(auto s:beam) {
                if(!s.sequence.empty()&&s.sequence.back().closing){children.push_back(s);continue;}
                auto opts=options(s.gate,s.direction,s.seen,s.length);
                if(opts.empty()){s.cost+=1;children.push_back(s);continue;}
                for(auto o:opts) {
                    auto child=s;child.cost+=o.cost;child.gate=o.gate;child.direction=o.unit;
                    child.seen[o.gate].push_back(o.unit);child.length+=o.ds;child.advance+=o.ds;child.sequence.push_back(o);
                    children.push_back(std::move(child));
                }
            }
            std::stable_sort(children.begin(),children.end(),[&](auto& a,auto& b){return score(a)<score(b);});
            if((int)children.size()>cfg.beam_width)children.resize(cfg.beam_width);beam=std::move(children);
        }
        auto winner=std::min_element(beam.begin(),beam.end(),[&](auto& a,auto& b){return score(a)<score(b);});
        require(winner!=beam.end()&&!winner->sequence.empty(),"Search produced no step");
        auto s=winner->sequence.front();committed.gate=s.gate;committed.direction=s.unit;
        committed.length+=s.ds;committed.seen[s.gate].push_back(s.unit);
        route.mid.push_back(gates[s.gate].mid);route.bridges+=s.bridge;
        if(s.closing){route.status="closed";return route;}
    }
    route.status="search_budget";return route;
}
// Dense pivoted solve for spline second derivatives. O(M^3), adequate for small
// teaching maps; replace with banded/cyclic solve for large competition maps.
std::vector<Vec2> solve(std::vector<std::vector<double>> a,std::vector<Vec2> b) {
    int n=(int)b.size();
    for(int k=0;k<n;++k) {
        int pivot=k;for(int i=k+1;i<n;++i)if(std::abs(a[i][k])>std::abs(a[pivot][k]))pivot=i;
        require(std::abs(a[pivot][k])>1e-14,"Singular spline system");
        std::swap(a[k],a[pivot]);std::swap(b[k],b[pivot]);
        for(int i=k+1;i<n;++i) {double f=a[i][k]/a[k][k];if(f==0)continue;
            for(int j=k;j<n;++j)a[i][j]-=f*a[k][j];b[i]=b[i]-b[k]*f;}
    }
    std::vector<Vec2> x(n);
    for(int i=n-1;i>=0;--i){Vec2 v=b[i];for(int j=i+1;j<n;++j)v=v-x[j]*a[i][j];x[i]=v/a[i][i];}
    return x;
}
struct Spline {
    std::vector<Vec2> p,m;std::vector<double> u;
    Spline(const std::vector<Vec2>& pts,bool closed):p(pts),u(pts.size(),0) {
        for(size_t i=1;i<p.size();++i){double h=norm(p[i]-p[i-1]);require(h>=1e-8,"Duplicate midpoint");u[i]=u[i-1]+h;}
        int n=(int)p.size()-(closed?1:0);
        std::vector<std::vector<double>> a(n,std::vector<double>(n,0));std::vector<Vec2> rhs(n);
        for(int i=0;i<n;++i) {
            if(!closed&&(i==0||i==n-1)){a[i][i]=1;continue;}
            int prev=(i+n-1)%n,next=(i+1)%n;
            double h0=i==0?u.back()-u[n-1]:u[i]-u[i-1];double h1=u[i+1]-u[i];
            a[i][prev]+=h0;a[i][i]+=2*(h0+h1);a[i][next]+=h1;
            rhs[i]=((p[next]-p[i])/h1-(p[i]-p[prev])/h0)*6;
        }
        m=solve(a,rhs);if(closed)m.push_back(m[0]);
    }
    std::array<Vec2,3> at(double q) const {
        int i=std::clamp((int)(std::upper_bound(u.begin(),u.end(),q)-u.begin())-1,0,(int)p.size()-2);
        double h=u[i+1]-u[i],a=(u[i+1]-q)/h,b=(q-u[i])/h;
        Vec2 value=p[i]*a+p[i+1]*b+(m[i]*(a*a*a-a)+m[i+1]*(b*b*b-b))*(h*h/6);
        Vec2 first=(p[i+1]-p[i])/h+(m[i]*(-3*a*a+1)+m[i+1]*(3*b*b-1))*(h/6);
        Vec2 second=m[i]*a+m[i+1]*b;return {value,first,second};
    }
};
// Directed gate hypotheses. Compare complete circuits by observed boundary coverage.
// Gates can cross the other branch; all physical cones remain obstacles during offset search.
bool properCross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    return orient(a,b,c)*orient(a,b,d)<0 && orient(c,d,a)*orient(c,d,b)<0;
}
int crossings(const std::vector<Vec2>& p) {
    int count=0;
    for(size_t i=1;i<p.size();++i) for(size_t j=i+2;j<p.size();++j) {
        if(i==1 && j+1==p.size())continue;
        if(properCross(p[i-1],p[i],p[j-1],p[j]))++count;
    }
    return count;
}
double segmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 d=b-a;
    double t=std::clamp(dot(p-a,d)/std::max(dot(d,d),1e-20),0.,1.);
    return norm(p-(a+d*t));
}
Route figureCircuit(const std::vector<Vec2>& blue,const std::vector<Vec2>& yellow,
                    Vec2 car,Vec2 heading,const Options& cfg) {
    std::vector<double> widths;
    for(auto b:blue) {
        double best=cfg.max_width;
        for(auto y:yellow)best=std::min(best,norm(b-y));
        widths.push_back(best);
    }
    std::sort(widths.begin(),widths.end());
    double w=widths[widths.size()/2];
    std::vector<Gate> gates;
    for(int i=0;i<(int)blue.size();++i) for(int j=0;j<(int)yellow.size();++j) {
        Vec2 d=yellow[j]-blue[i]; double width=norm(d);
        if(width<std::max(cfg.min_width,.75*w)||width>std::min(cfg.max_width,1.3*w))continue;
        gates.push_back({{i,j},(blue[i]+yellow[j])/2,{-d.y/width,d.x/width},width,{}});
    }
    require(!gates.empty()&&gates.size()<=2500,"Figure-eight gate budget or width mismatch");
    std::sort(gates.begin(),gates.end(),[](const Gate& a,const Gate& b){
        return std::tie(a.mid.x,a.mid.y,a.tangent.x,a.tangent.y)<std::tie(b.mid.x,b.mid.y,b.tangent.x,b.tangent.y);
    });
    int start=-1;double best=1e30;
    for(int i=0;i<(int)gates.size();++i) {
        const auto& g=gates[i]; Vec2 d=g.mid-car; double align=dot(g.tangent,heading);
        if(dot(d,heading)<-1e-7||align<.7||norm(d)>cfg.max_start_distance)continue;
        double score=norm(d)+w*(1-align)+2*std::abs(g.width-w);
        if(score<best){best=score;start=i;}
    }
    require(start>=0,"No oriented start gate");
    struct Link { int to; double cost; };
    std::vector<std::vector<Link>> links(gates.size());
    for(size_t i=0;i<gates.size();++i)for(size_t j=0;j<gates.size();++j) {
        const auto& a=gates[i]; const auto& b=gates[j]; Vec2 d=b.mid-a.mid;double ds=norm(d);
        if(ds<.05||ds>2.6*w||a.ids.first==b.ids.first||a.ids.second==b.ids.second)continue;
        Vec2 u=d/ds; double align=std::min(dot(u,a.tangent),dot(u,b.tangent));
        if(align<.74 || dot(a.tangent,b.tangent)<.25)continue;
        double error=norm(u*2-a.tangent-b.tangent);
        double cost=2*error+2*std::abs(b.width-w)/w+.45*ds/w;
        links[i].push_back({(int)j,cost});
    }
    struct State { std::vector<int> path; std::set<int> left,right; double cost; };
    State first{{start},{gates[start].ids.first},{gates[start].ids.second},0};
    std::vector<State> beam{first}; Route winner; size_t bestCoverage=0; double bestCost=1e30;
    for(size_t depth=0;depth<=std::min(blue.size(),yellow.size())&&!beam.empty();++depth) {
        std::vector<State> next;
        for(const auto& s:beam) for(auto link:links[s.path.back()]) {
            const auto& g=gates[link.to];double cost=s.cost+link.cost;
            if(link.to==start && s.path.size()>=6) {
                if(s.path.size()<bestCoverage || (s.path.size()==bestCoverage&&cost>=bestCost))continue;
                std::vector<Vec2> p;for(int k:s.path)p.push_back(gates[k].mid);
                p.push_back(p.front());if(crossings(p)!=1)continue;
                bestCoverage=s.path.size();bestCost=cost;winner.mid=std::move(p);winner.status="closed";
            } else if(!s.left.count(g.ids.first)&&!s.right.count(g.ids.second)) {
                State child=s;child.path.push_back(link.to);child.left.insert(g.ids.first);
                child.right.insert(g.ids.second);child.cost=cost;next.push_back(std::move(child));
            }
        }
        std::stable_sort(next.begin(),next.end(),[](const State& a,const State& b){return a.cost<b.cost;});
        if(next.size()>32)next.resize(32);
        beam=std::move(next);
    }
    require(bestCoverage>=.65*std::min(blue.size(),yellow.size()),"No supported two-lobe circuit");
    return winner;
}
// Shortest path through lateral-offset samples of the midpoint spline. Each edge
// clears every measured cone. Keep the start fixed and explicitly close the lap.
std::vector<Vec2> offsetCircuit(const Spline& spline,const std::vector<Vec2>& cones,double clearance) {
    int n=(int)std::ceil(spline.u.back()/.4);
    require(n>=8&&n<2000,"Figure-eight offset search budget exceeded");
    constexpr int radius=14, lanes=2*radius+1;
    using Row=std::array<Vec2,lanes>;
    std::vector<Row> layer(n+1);std::vector<std::array<int,lanes>> previous(n+1);
    std::array<double,lanes> cost;cost.fill(1e30);cost[radius]=0;
    for(int i=0;i<=n;++i) {
        auto q=spline.at(spline.u.back()*(i==n?0:i)/n);
        require(norm(q[1])>1e-7,"Undefined centreline tangent");
        Vec2 normal={-q[1].y/norm(q[1]),q[1].x/norm(q[1])};
        for(int k=0;k<lanes;++k)layer[i][k]=q[0]+normal*((k-radius)*.08);
        if(i==0)continue;
        std::vector<Vec2> nearby;
        for(auto c:cones)if(segmentDistance(c,layer[i-1][radius],q[0])<=radius*.08+clearance+.05)nearby.push_back(c);
        std::array<double,lanes> next;next.fill(1e30);
        for(int k=0;k<lanes;++k)for(int j=std::max(0,k-4);j<=std::min(lanes-1,k+4);++j) {
            if(cost[j]>=1e30)continue;
            double candidate=cost[j]+(k-radius)*(k-radius)+10*(k-j)*(k-j);
            if(candidate>=next[k])continue;
            bool clear=true;for(auto c:nearby)if(segmentDistance(c,layer[i-1][j],layer[i][k])<clearance+.04){clear=false;break;}
            if(clear){next[k]=candidate;previous[i][k]=j;}
        }
        cost=next;
    }
    require(cost[radius]<1e30,"No cone-clear offset path at requested clearance");
    std::vector<Vec2> path(n+1);int k=radius;
    for(int i=n;i>=0;--i){path[i]=layer[i][k];if(i)k=previous[i][k];}
    return path;
}

}
Path plan(const std::vector<Vec2>& blue,const std::vector<Vec2>& yellow,Vec2 car,Vec2 heading,const Options& cfg) {
    require(blue.size()>=2&&yellow.size()>=2,"At least two cones per colour required");
    require(finite(car)&&finite(heading)&&norm(heading)>1e-9,"Invalid pose");heading=heading/norm(heading);
    for(double x:{cfg.spacing,cfg.dense_step,cfg.min_width,cfg.max_width,cfg.max_start_distance,cfg.max_turn_deg})require(std::isfinite(x)&&x>0,"Invalid options");
    require(cfg.min_width<cfg.max_width&&cfg.max_turn_deg<180&&cfg.lookahead>=1&&cfg.lookahead<=10&&cfg.beam_width>=1&&cfg.beam_width<=32,"Invalid options");
    std::vector<Vec2> cones=blue;cones.insert(cones.end(),yellow.begin(),yellow.end());
    require(cones.size()<=5000,"Experimental map budget exceeded");std::set<std::pair<double,double>> unique;
    for(auto p:cones)require(finite(p)&&unique.insert({p.x,p.y}).second,"Invalid or duplicate cone");
    auto tris=triangulate(cones);std::set<Edge> edges;
    for(auto t:tris)for(int k=0;k<3;++k){auto e=edge(t[k],t[(k+1)%3]);double w=norm(cones[e.first]-cones[e.second]);
        if((e.first<(int)blue.size())!=(e.second<(int)blue.size())&&w>=cfg.min_width&&w<=cfg.max_width)edges.insert(e);}
    std::vector<Gate> gates;std::map<Edge,int> ids;
    for(auto e:edges){Vec2 d=cones[e.second]-cones[e.first];double w=norm(d);ids[e]=(int)gates.size();gates.push_back({e,(cones[e.first]+cones[e.second])/2,{-d.y/w,d.x/w},w,{}});}
    for(auto t:tris){std::vector<int> valid;for(int k=0;k<3;++k){auto it=ids.find(edge(t[k],t[(k+1)%3]));if(it!=ids.end())valid.push_back(it->second);}
        if(valid.size()==2){gates[valid[0]].neighbours.insert(valid[1]);gates[valid[1]].neighbours.insert(valid[0]);}}
    auto route=cfg.figure_clearance>0 ? figureCircuit(blue,yellow,car,heading,cfg) : select(cones,gates,car,heading,cfg);
    require(!cfg.closed||route.status=="closed","One-lap route did not close");require(route.mid.size()>=4,"Fewer than four connected gates");
    require(route.mid.size()<=2000,"Experimental dense spline budget exceeded");
    std::vector<Vec2> knots=route.mid;
    if(cfg.figure_clearance>0)knots=offsetCircuit(Spline(route.mid,true),cones,cfg.figure_clearance);
    require(knots.size()<=2000,"Spline budget exceeded");
    Spline spline(knots,cfg.closed);Path out;out.midpoints=route.mid;out.status=route.status;out.bridge_count=route.bridges;out.closed=cfg.closed;out.requested_spacing=cfg.spacing;
    double count=std::max(100.,std::ceil(spline.u.back()/cfg.dense_step)+1);
    require(count<=1000000,"Dense sample budget exceeded");int n=(int)count;std::vector<double> arc(n,0);
    for(int i=0;i<n;++i){auto v=spline.at(spline.u.back()*i/(n-1));out.dense_points.push_back(v[0]);
        out.signed_curvature.push_back((v[1].x*v[2].y-v[1].y*v[2].x)/std::pow(std::max(dot(v[1],v[1]),1e-20),1.5));
        if(i)arc[i]=arc[i-1]+norm(v[0]-out.dense_points[i-1]);}
    out.arc_length=arc.back();require(out.arc_length>=cfg.spacing,"Path shorter than spacing");
    double segments=cfg.closed?std::max(3.,std::ceil(arc.back()/cfg.spacing)):std::floor(arc.back()/cfg.spacing);
    require(segments<=1000000,"Output sample budget exceeded");int number=(int)segments;
    out.actual_arc_spacing=cfg.closed?arc.back()/number:cfg.spacing;
    for(int i=0;i<=number;++i){double s=i*out.actual_arc_spacing;int j=std::clamp((int)(std::upper_bound(arc.begin(),arc.end(),s)-arc.begin())-1,0,n-2);
        double f=(s-arc[j])/(arc[j+1]-arc[j]);out.points.push_back(out.dense_points[j]*(1-f)+out.dense_points[j+1]*f);}
    if(cfg.closed)out.points.back()=out.points.front();
    for(size_t i=1;i<out.points.size();++i)out.vectors.push_back(out.points[i]-out.points[i-1]);
    if(cfg.figure_clearance>0) {
        require(crossings(out.points)==1,"Output does not cross exactly once");
        for(size_t i=1;i<out.points.size();++i)for(auto c:cones)
            require(segmentDistance(c,out.points[i-1],out.points[i])>=cfg.figure_clearance,"Output cuts cone clearance");
    }
    out.omitted_tail=std::max(0.,out.arc_length-number*out.actual_arc_spacing);return out;
}
RouteProgress::RouteProgress(const std::vector<Vec2>& points,int target,double speed,double error,double window):points_(points),target_(target),max_speed_(speed),max_error_(error),window_(window) {
    require(points.size()>=5&&target>=1,"Invalid route or target laps");for(auto p:points)require(finite(p),"Nonfinite route");
    require(norm(points.front()-points.back())<=1e-8,"Route must explicitly close");
    for(double v:{speed,error,window})require(std::isfinite(v)&&v>0,"Invalid tracking limits");
    arc_.push_back(0);for(size_t i=1;i<points.size();++i){auto d=points[i]-points[i-1];double l=norm(d);require(l>=1e-8,"Zero route segment");delta_.push_back(d);lengths_.push_back(l);arc_.push_back(arc_.back()+l);}
    require(window<length()/4,"Search window must be shorter than quarter lap");
}
Progress RouteProgress::state() const {
    int laps=std::min(target_,(int)std::floor((distance_+1e-8)/length()));
    return {distance_,std::fmod(distance_,length()),error_,laps,laps>=target_};
}
Progress RouteProgress::update(Vec2 p,Vec2 h,double dt) {
    require(finite(p)&&finite(h)&&norm(h)>1e-9&&std::isfinite(dt)&&dt>0,"Invalid pose or dt");h=h/norm(h);
    if(distance_>=target_*length())return state();double budget=max_speed_*dt+.15;
    require(budget<=window_,"Update interval too long; explicit relocalisation required");
    require(!has_position_||norm(p-last_position_)<=budget,"Pose jump exceeds movement budget");
    double lo=std::max(0.,distance_-.5),hi=std::min(distance_+budget,length()*target_);
    auto best=std::make_tuple(std::numeric_limits<double>::infinity(),0.,0.);
    for(int lap=(int)std::floor(lo/length());lap<=(int)std::floor(hi/length());++lap)for(size_t j=0;j<delta_.size();++j){
        double start=arc_[j]+lap*length(),end=arc_[j+1]+lap*length();if(end<lo||start>hi)continue;
        auto tangent=delta_[j]/lengths_[j];if(dot(tangent,h)<.5)continue;
        double f=std::clamp(dot(p-points_[j],delta_[j])/(lengths_[j]*lengths_[j]),0.,1.),s=start+f*lengths_[j];
        if(s<lo-1e-8||s>hi+1e-8)continue;
        double e=norm(p-(points_[j]+delta_[j]*f));best=std::min(best,std::make_tuple(e+.02*(1-dot(tangent,h)),e,s));
    }
    auto [score,e,s]=best;require(std::isfinite(score),"No direction-consistent route match");require(e<=max_error_,"Pose too far from expected route");
    distance_=std::max(distance_,std::min(s,target_*length()));error_=e;last_position_=p;has_position_=true;return state();
}
}

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

void points(const std::vector<fsai::Vec2>& p) {
    std::cout<<'[';for(size_t i=0;i<p.size();++i){if(i)std::cout<<',';std::cout<<'['<<p[i].x<<','<<p[i].y<<']';}std::cout<<']';
}
void read_point(fsai::Vec2& p) { if(!(std::cin>>p.x>>p.y))throw std::runtime_error("Malformed point input"); }
int main(int argc,char** argv) {
    std::cout<<std::setprecision(17);
    try {
        if(argc==2&&std::string(argv[1])=="--track") {
            int n,target,updates;if(!(std::cin>>n>>target>>updates)||n<5||n>1000001||updates<0||updates>1000000)throw std::runtime_error("Invalid tracking header");
            std::vector<fsai::Vec2> p(n);for(auto& q:p)read_point(q);
            fsai::RouteProgress tracker(p,target);
            for(int i=0;i<updates;++i) {
                fsai::Vec2 pos,h;double dt;read_point(pos);read_point(h);if(!(std::cin>>dt))throw std::runtime_error("Missing dt");
                bool accepted=true;try{tracker.update(pos,h,dt);}catch(const std::invalid_argument&){accepted=false;}
                auto s=tracker.state();std::cout<<"{\"accepted\":"<<(accepted?"true":"false")<<",\"distance_m\":"<<s.distance<<",\"completed_laps\":"<<s.completed_laps<<",\"finished\":"<<(s.finished?"true":"false")<<"}\n";
            }return 0;
        }
        bool eight=argc==3 && std::string(argv[1])=="--figure8";
        if(argc!=1&&!eight)throw std::runtime_error("Usage: fsai_centerline < map.txt OR fsai_centerline --track < poses.txt");
        int nb,ny,closed;if(!(std::cin>>nb>>ny>>closed)||nb<2||ny<2||nb>5000 || ny>5000 || nb+ny>5000||(closed!=0&&closed!=1))throw std::runtime_error("Invalid map header");
        fsai::Options options;options.closed=closed;
        if(eight) {
            size_t consumed=0;
            options.figure_clearance=std::stod(argv[2],&consumed);
            if(!closed||consumed!=std::string(argv[2]).size()||!std::isfinite(options.figure_clearance)||options.figure_clearance<=0)
                throw std::runtime_error("Figure-eight requires a closed map and positive cone-centre clearance in metres");
        }
        if(!(std::cin>>options.spacing))throw std::runtime_error("Missing spacing");
        fsai::Vec2 car,heading;read_point(car);read_point(heading);
        std::vector<fsai::Vec2> blue(nb),yellow(ny);for(auto& p:blue)read_point(p);for(auto& p:yellow)read_point(p);
        auto r=fsai::plan(blue,yellow,car,heading,options);
        std::cout<<"{\"frame\":\"map\",\"units\":\"m\",\"closed\":"<<(r.closed?"true":"false")<<",\"status\":\""<<r.status<<"\",\"bridge_count\":"<<r.bridge_count
                 <<",\"requested_spacing_m\":"<<r.requested_spacing<<",\"actual_arc_spacing_m\":"<<r.actual_arc_spacing<<",\"arc_length\":"<<r.arc_length<<",\"omitted_tail_m\":"<<r.omitted_tail;
        std::cout<<",\"start_position\":["<<r.points.front().x<<','<<r.points.front().y<<']';
        std::cout<<",\"points\":";points(r.points);std::cout<<",\"vectors\":";points(r.vectors);
        std::cout<<",\"midpoints\":";points(r.midpoints);std::cout<<",\"dense_points\":";points(r.dense_points);
        std::cout<<",\"signed_curvature\":[";for(size_t i=0;i<r.signed_curvature.size();++i){if(i)std::cout<<',';std::cout<<r.signed_curvature[i];}std::cout<<"]}\n";
    }catch(const std::exception& e){std::cerr<<"Unavailable: "<<e.what()<<'\n';return 2;}
}

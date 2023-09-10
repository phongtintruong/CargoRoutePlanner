#include <bits/stdc++.h>

#define ll long long
#define el '\n'
#define pii pair<int, int>
#define ppi pair<pii, int>
#define mp(a, b) make_pair(a, b)
#define pb(a) push_back(a)
#define INF 1e9+7
#define FI first
#define SE second

// #define TASK "BFShipper"
// #define cin f1
// #define cout f2

using namespace std;

const int maxC = 1500;
const int maxIter = 25000;

struct Query
{
    int s, e;
    double d, v;
    int sp, sd;
    pii s_wd; // Hub s time window for task
    pii e_wd; // Hub e time window for task
    int id; // Absolute id
};

struct Truck
{
    int p;
    pii wd; // Working window for Truck
    double c, vol;
    double vel;
    int id; // Absolute id
};

struct Truck_state
{
    int cur_time = 0;
    int cur_hub = 0;
    int prev_hub = -1;
    int n_package = 0;
    double cur_weight = 0;
    double cur_volumn = 0;
};

// ifstream f1("E:\\BKAI\\Routing\\BFShipper.INP");
// ofstream f2("E:\\BKAI\\Routing\\BFShipper.OUT");

int M, N, K;
double d[maxC][maxC];
int trace[maxC][maxC];
Truck truck[maxC];
Query q[maxC];
Query best_q[maxC];

double maxD = -1;
int maxT = -1;

vector<int> truck_q[maxC];
vector<int> best_truck_q[maxC];
int q_mask[maxC], truck_mask[maxC];

mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

// UTILITIES //
int str2int(string s)
{
    int res = 0;
    for(int i=0; i<s.length(); ++i) res = res * 10 + (s[i] - '0');
    return res;
}

string int2str(int x)
{
    string s = "";
    while(x > 0)
    {
        s = (char) (x % 10 + 48) + s;
        x /= 10;
    }
    while(s.length() < 2) s = '0' + s;
    return s;
}

int hms2sec(string hms)
{
    string hh = hms.substr(0, 2);
    string mm = hms.substr(3, 2);
    string ss = hms.substr(6, 2);

    int h = str2int(hh);
    int m = str2int(mm);
    int s = str2int(ss);

    return h * 3600 + m * 60 + s;
}

string sec2hmr(int sec)
{
    int h = sec / 3600;
    sec -= h * 3600;
    int m = sec / 60;
    sec -= m * 60;
    int s = sec;

    return int2str(h) + ":" + int2str(m) + ":" + int2str(s);
}

double Abs(double a)
{
    return (a < 0) ? (-a) : a;
}

bool Equal(double a, double b, double epsi = 1e-8)
{
    return (Abs(a - b) <= epsi);
}

double Q_METRIC(Query q)
{
    return (double) q.e_wd.SE / maxT + 0.01 * (double) d[q.s][q.e] / maxD;
}

bool SORT_Q(Query q1, Query q2)
{
    return (Q_METRIC(q1) < Q_METRIC(q2));
    // return (q1.s_wd.FI < q2.s_wd.FI || (q1.s_wd.FI == q2.s_wd.FI && q1.e_wd.SE < q2.e_wd.SE));
}

bool SORT_TRUCK(Truck t1, Truck t2)
{
    if(t1.wd.FI != t2.wd.FI) return (t1.wd.FI > t2.wd.FI);
    if(t1.wd.SE != t2.wd.SE) return (t2.wd.SE > t2.wd.SE);
    if(t1.vel != t2.vel) return (t1.vel > t2.vel);
    if(t1.c != t2.c) return (t1.c > t2.c);
    return (t1.vol > t2.vol);
}

bool Check_truck_q(Truck t, Query q, Truck_state ts)
{
    if(q.d > t.c || q.v > t.vol) return false;

    // Arrive time constraint
    int ar = ts.cur_time + ceil(d[ts.cur_hub][q.s] / t.vel); // Arrive time at s
    if(ar > q.s_wd.SE) return false;

    // Depart time constraint
    int de = max(ar, q.s_wd.FI) + q.sp + ceil(d[q.s][q.e] / t.vel); // Depart time at e
    if(de > q.e_wd.SE) return false;

    // End-of-truck-shift constraint:
    int fde = max(de, q.e_wd.FI) + q.sd + ceil(d[q.e][t.p] / t.vel); // Final departure at base hub
    if(fde > t.wd.SE) return false;
    
    return true;
}

int GetFinTime(Truck t, Query q, Truck_state ts)
{
    int tmp = ts.cur_time;
    
    tmp += ceil(d[ts.cur_hub][q.s] / t.vel);
    tmp = max(ts.cur_time, q.s_wd.FI);
    tmp += q.sp;

    tmp += ceil(d[q.s][q.e] / t.vel);
    tmp = max(ts.cur_time, q.e_wd.FI);
    tmp += q.sd;
    
    return tmp;
}

vector<int> Trace(int u, int v)
{
    vector<int> route;
    int cur = u;
    while(cur != v)
    {
        route.pb(cur);
        cur = trace[cur][v];
    }
    route.pb(v);
    return route;
}
// END OF UTLILITIES //

// INITIALIZE //
void Input()
{
    cin >> M;
    for(int i=1; i<=M; ++i)
        for(int j=1; j<=M; ++j) 
        {
            cin >> d[i][j];
            trace[i][j] = j;
        }

    cin >> K;
    for(int i=1; i<=K; ++i)
    {
        truck[i].id = i;

        cin >> truck[i].p;

        string F, T;
        cin >> F >> T;
        truck[i].wd = mp(hms2sec(F), hms2sec(T));
        
        cin >> truck[i].c >> truck[i].vol >> truck[i].vel;
        truck[i].vel = (double) (truck[i].vel / 3600.0);
    }

    cin >> N;
    for(int i=1; i<=N; ++i)
    {
        q[i].id = i;
        
        cin >> q[i].s >> q[i].e;
        cin >> q[i].d >> q[i].v;
        cin >> q[i].sp >> q[i].sd;
        
        string E, L;
        cin >> E >> L;
        q[i].s_wd = mp(hms2sec(E), hms2sec(L));

        cin >> E >> L;
        q[i].e_wd = mp(hms2sec(E), hms2sec(L));
        
        maxT = max(maxT, hms2sec(L));
    }

    return;
}

void Floyd()
{
    for(int k=1; k<=M; ++k)
    {
        for(int i=1; i<=M; ++i)
        {
            for(int j=1; j<=M; ++j)
            {
                if(d[i][j] > d[i][k] + d[k][j])
                {
                    d[i][j] = d[i][k] + d[k][j];
                    trace[i][j] = trace[i][k];
                }
            }
        }
    }
    
    for(int i=1; i<=M; ++i)
        for(int j=1; j<=M; ++j) maxD = max(maxD, d[i][j]);
        
    return;
}

void Init()
{
    Input();
    Floyd();
    sort(q+1, q+1+N, SORT_Q);
    sort(truck+1, truck+1+K, SORT_TRUCK);
    return;
}
// END OF INITIALIZE //

// GENERATOR //
int Rand(int l,int r)
{
	return l + rng() % (r-l+1);
}

void Q_ptb(int cnt)
{
    // Q - perturbation
    for(int i=1; i<=cnt; ++i)
    {
        int l = Rand(N/4, N/2);
        int r = Rand(3*N/4, N);
        swap(q[l], q[r]);
    }
    return;
}
// END OF GENERATOR //

// EVALUATE //
void Clear()
{
    memset(q_mask, 0, sizeof(q_mask));
    memset(truck_mask, 0, sizeof(truck_mask));
    for(int i=1; i<=K; ++i) truck_q[i].clear();
    return;
}

int GetLoss()
{
    int cnt = 0;
    for(int i=1; i<=N; ++i) cnt += (q_mask[i] == 1);
    return -cnt;
}
// END OF EVALUATE //

// SOLUTION //
void Solve()
{
    while(true)
    {
        int q_id = -1; // Next query to be solved
        for(int i=1; i<=N; ++i) 
        {
            if(q_mask[i] == 0)
            {
                q_id = i;
                break;
            }
        }
        if(q_id == -1) break;

        int truck_id = -1;
        Truck_state ts;
        int best_fintime = 1e5;
        for(int i=1; i<=K; ++i)
        {
            ts.cur_time = truck[i].wd.FI;
            ts.cur_hub = truck[i].p;
            if(truck_mask[i] == 0 && Check_truck_q(truck[i], q[q_id], ts) && GetFinTime(truck[i], q[q_id], ts) < best_fintime)
            {
                truck_id = i;
                best_fintime = GetFinTime(truck[i], q[q_id], ts);
                break;
            }
        }

        if(truck_id == -1)
        {
            q_mask[q_id] = -1;
            continue;
        }

        Query cur_q = q[q_id];
        Truck cur_truck = truck[truck_id];

        while(true)
        {
            cur_q = q[q_id];
            q_mask[q_id] = 1;

            truck_q[truck_id].pb(q_id);

            // Update truck state
            ts.cur_time += ceil(d[ts.cur_hub][cur_q.s] / cur_truck.vel);
            ts.cur_time = max(ts.cur_time, cur_q.s_wd.FI);
            ts.cur_time += cur_q.sp;

            ts.cur_time += ceil(d[cur_q.s][cur_q.e] / cur_truck.vel);
            ts.cur_time = max(ts.cur_time, cur_q.e_wd.FI);
            ts.cur_time += cur_q.sd;

            ts.cur_hub = q[q_id].e;

            // Find next mission
            bool flag = false;
            for(int i=1; i<=N; ++i)
            {
                if(q_mask[i] == 0 && Check_truck_q(cur_truck, q[i], ts))
                {
                    flag = true;
                    q_id = i;
                    q_mask[i] = 1;
                    break;
                }
            }
            
            if(!flag)
            {
                truck_mask[truck_id] = 1;
                break;
            }
        }
    }
}

void UpdateSol()
{
    copy(q+1, q+1+N, best_q+1);
    for(int i=1; i<=K; ++i)
    {
        best_truck_q[i].clear();
        for(int j=0; j<truck_q[i].size(); ++j) best_truck_q[i].pb(truck_q[i][j]);
    }
}

void MultiSolve()
{
    Solve();
    int best_loss = GetLoss();
    UpdateSol();
    Clear();
    // cerr << best_loss << el;

    for(int i=1; i<=maxIter; ++i)
    {
        copy(best_q+1, best_q+1+N, q+1);
        int cnt = Rand(5, 10);
        Q_ptb(cnt);
        Solve();
        
        int new_loss = GetLoss();
        if(new_loss < best_loss) 
        {
            best_loss = new_loss;
            // cerr << new_loss << el;
            UpdateSol();
        }

        Clear();
    }

    copy(best_q+1, best_q+1+N, q+1);
    for(int i=1; i<=K; ++i)
    {
        truck_q[i].clear();
        for(int j=0; j<best_truck_q[i].size(); ++j) truck_q[i].pb(best_truck_q[i][j]);
    }
}
// END OF SOLUTION //

// EVALUATE //
void Print_truck_sol(int id)
{
    Truck cur_truck = truck[id];
    int u = 0;
    for(int i=0; i<truck_q[id].size(); ++i)
    {
        Query cur_q = q[truck_q[id][i]];
        vector<int> route = Trace(cur_q.s, cur_q.e);
        u += route.size();
    }
    
    if(u == 0)
    {
        cout << 1 << el;
        cout << truck[id].p << ' ' << 0 << ' ' << sec2hmr(truck[id].wd.FI) << ' ' << sec2hmr(truck[id].wd.FI) << el;
        return;
    }

    cout << u+2 << el;

    Truck_state ts;
    ts.cur_time = truck[id].wd.FI;
    ts.cur_hub = truck[id].p;

    // Begin shift
    cout << ts.cur_hub << ' ' << 0 << ' ' << sec2hmr(ts.cur_time) << ' ' << sec2hmr(ts.cur_time);
    cout << el;

    for(int i=0; i<truck_q[id].size(); ++i)
    {
        Query cur_q = q[truck_q[id][i]];
        vector<int> route = Trace(cur_q.s, cur_q.e);

        int lroute = route.size();
        for(int j=0; j<lroute; ++j)
        {
            cout << route[j] << ' ';

            if(j == 0)
            {
                cout << 1 << ' ';

                int ar = ts.cur_time + ceil(d[ts.cur_hub][route[j]] / cur_truck.vel);
                cout << sec2hmr(max(ar, cur_q.s_wd.FI)) << ' ';
                
                int de = max(ar, cur_q.s_wd.FI) + cur_q.sp;
                cout << sec2hmr(de) << el;

                cout << q[truck_q[id][i]].id << ' ' << sec2hmr(max(ar, cur_q.s_wd.FI)) << el;

                ts.cur_time = de;
                ts.cur_hub = route[j];
                continue;
            }

            if(j == lroute-1)
            {
                cout << 1 << ' ';

                int ar = ts.cur_time + ceil(d[ts.cur_hub][route[j]] / cur_truck.vel);
                cout << sec2hmr(max(ar, cur_q.e_wd.FI)) << ' ';

                int de = max(ar, cur_q.e_wd.FI) + cur_q.sd;
                cout << sec2hmr(de) << el;

                cout << q[truck_q[id][i]].id << ' ' << sec2hmr(max(ar, cur_q.e_wd.FI)) << el;

                ts.cur_time = de;
                ts.cur_hub = route[j];
                continue;
            }

            cout << 0 << ' ';

            int ar = ts.cur_time + ceil(d[ts.cur_hub][route[j]] / cur_truck.vel);
            cout << sec2hmr(ar) << ' ' << sec2hmr(ar) << el;

            ts.cur_time = ar;
            ts.cur_hub = route[j];
        }
    }

    // End shift
    int ar = ts.cur_time + ceil(d[ts.cur_hub][cur_truck.p] / cur_truck.vel);
    cout << cur_truck.p << ' ' << 0 << ' ' << sec2hmr(ar) << ' ' << sec2hmr(ar) << el;

    return;
}

void PrintResult()
{
    for(int i=1; i<=K; ++i) 
    {
        for(int j=1; j<=K; ++j)
        {
            if(truck[j].id == i) Print_truck_sol(j);
        }
    }
    return;
}
// END OF EVALUATE //

void Debug()
{
    vector<int> tmp = Trace(3, 1);
    for(int i=0;i<tmp.size();++i) cerr<<tmp[i]<<' ';
}

int main()
{
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    cout.tie(NULL);
    Init();
    MultiSolve();
    PrintResult();
    return 0;
}
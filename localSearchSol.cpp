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

#define TASK "BFShipper"
#define cin f1
#define cout f2

using namespace std;

const int maxC = 1500;
const int TIME_LIMIT = 60;
const int NUM_OPS = 2;

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

ifstream f1("path:\\to\\folder\\LocalShipper.INP");
ofstream f2("path:\\to\\folder\\Localshipper.OUT");

string OPS[NUM_OPS] = {"swap", "insert"};

int M, N, K;
double d[maxC][maxC];
int trace[maxC][maxC];
Truck truck[maxC];
Query q[maxC];
Query best_q[maxC];

int Next[maxC * 2], Prev[maxC * 2], Root[maxC * 2];
int bestNext[maxC * 2], bestPrev[maxC * 2], bestRoot[maxC * 2];
double bestScore = -1;

double maxD = -1;
int maxT = -1;

mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());
clock_t tStart;

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

bool CheckTime()
{
    return ((ceil((clock() - tStart) / CLOCKS_PER_SEC)) <= TIME_LIMIT);
}

double Abs(double a)
{
    return (a < 0) ? (-a) : a;
}

bool Equal(double a, double b, double epsi = 1e-8)
{
    return (Abs(a - b) <= epsi);
}

int Rand(int l,int r)
{
	return l + rng() % (r-l+1);
}

int getID(int node_id)
{
    if(node_id > 2 * (N + K)) return node_id;
    else return (node_id - N - K);
}

void Clear()
{
    /*
    Initialize curent solution with best solution 
    */

    for(int i=1; i<=((N+K) << 1); ++i)
    {
        Next[i] = bestNext[i];
        Prev[i] = bestPrev[i];
        Root[i] = bestRoot[i];
    }
    return;
}

void UpdateSol()
{
    /*
    Initialize curent solution with best solution 
    */

    for(int i=1; i<=((N+K) << 1); ++i)
    {
        bestNext[i] = Next[i];
        bestPrev[i] = Prev[i];
        bestRoot[i] = Root[i];
    }
    return;
}
// END OF UTILS //

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

void Init()
{
    Input();
    // Initialize (Next[], Prev[], Root[]) with an initial solution //
    memset(Root, -1, sizeof(Root));
    return;
}
// END OF INITIALIZE //

// OPERATORS //
void Swap(int u, int v)
{
    /*
    Swap two queries qu and qv.

    Input: 
    -----
    (Next[], Prev[], Root[]): Current best solution.
    - No need for specified input solution, as "Next, Prev, Root" are global variable.
    - Swapped queries with INDEX: u, v. The queries can be performed same-truck or cross-truck.

    Output:
    -----
    Transformed (Next[], Prev[], Root[])
    - The operations will directly affect global variables "Next, Prev, Root".
    */

   Query qu = q[u];
   Query qv = q[v];

   // YOUR CODE HERE //

   return;
}

void Insert(int q_id, int t_id)
{
    /*
    Insert query q into truck t's schedule. Among all possible insertion, choose the best insertion.

    Input: 
    -----
    (Next[], Prev[], Root[]): Current best solution.
    - No need for specified input solution, as "Next, Prev, Root" are global variable.
    - Query index q_id, Truck index t_id.

    Output:
    -----
    Transformed (Next[], Prev[], Root[])
    - The operations will directly affect global variables "Next, Prev, Root".
    */

   Query cur_q = q[q_id];
   Truck cur_truck = truck[t_id];

   return;
}
// END OF OPERATORS //

// EVALUATE //
bool CheckValidity()
{
    /* Check validity of current solution */
    return;
}

int countQ()
{
    /* Count the number of served query */
    return 0;
}

int countTruck()
{
    /* Count the number of served truck */
    return 0;
}

int countTime()
{
    /* Count the total runtim of K trucks */
    return 0;
}

int GetScore()
{
    int q_cnt = countQ();
    int truck_cnt = countTruck();
    int t_cnt = countTime();
    // return (double) 1e9 * (double) q_cnt / N - (double) 1e6 * (double) truck_cnt / K + (double) t_cnt / (1e3);
    return q_cnt; // Assign scores based on only the number of served query. Recommended for development phase. 
}
// END OF EVALUATE //

// LOCAL SEARCH // 
void Local_Search()
{
    while(CheckTime())
    {
        int op_id = Rand(0, NUM_OPS-1);
        string op = OPS[op_id];
        
        if(op == "swap")
        {
            int u = Rand(1, N);
            int v = Rand(min(u+1, N), N);
            while(Root[u] == -1 && Root[v] == -1)
            {
                int u = Rand(1, N);
                int v = Rand(min(u+1, N), N);
            }
            Swap(u, v);
        }

        if(op == "insert")
        {
            int q_id = -1;
            for(int i=1; i<=N; ++i)
            {
                if(!CheckTime()) break;
                if(Root[i] == -1) 
                {
                    q_id = i;
                    break;
                }
            }
            if(q_id == -1) break;
            int t_id = Rand(1, K);
            Insert(q_id, t_id);
        }

        double score = GetScore();
        if(score > bestScore && CheckValidity()) 
        {
            bestScore = score;
            UpdateSol();
        }

        Clear();
    }
}
// END OF LOCAL SEARCH //

// LOG //
void printLog()
{
    /* 
    Print the log according to problem description. 
    A 'Truck_state' is recommended to be used for tracking activites 
    */
    return;
}
// END OF LOG //

void Solve()
{
    Local_Search();
    printLog();
}

int main()
{
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    cout.tie(NULL);
    tStart = clock();
    Init();
    Solve();
    return 0;
}
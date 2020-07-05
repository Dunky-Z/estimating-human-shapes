#include "convex_hull.h"

ConvexHull::ConvexHull()
{
}

ConvexHull::~ConvexHull()
{
}


int ConvexHull::orientation(pmp::vec3 p, pmp::vec3 q, pmp::vec3 r)
{
	int val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
	if (val == 0) return 0;  // Collinear
	return (val > 0) ? -1 : 1; // CW: -1 or CCW: 1
}

/*
	Predicate function used while sorting the points using qsort() inbuilt function in C++
	@param p: Object of class point aka first point
	@param p: Object of class point aka second point
*/
int ConvexHull::compare(const pmp::vec3 p1, const pmp::vec3 p2)
{
	int orient = orientation(ori_point, p1, p2);
	if (orient == 0)
		return (distance(ori_point, p2) >= distance(ori_point, p1)) ? -1 : 1;
	return (orient == 1) ? -1 : 1;
}

/*
	Returns the index of the point to which the tangent is drawn from point p.
	Uses a modified Binary Search Algorithm to yield tangent in O(log n) complexity
	@param v: vector of objects of class points representing the hull aka the vector of hull points
	@param p: Object of class point from where tangent needs to be drawn
*/
int ConvexHull::tangent(vector<pmp::vec3> v, pmp::vec3 p)
{
	int l = 0;
	int r = v.size();
	int l_before = orientation(p, v[0], v[v.size() - 1]);
	int l_after = orientation(p, v[0], v[(l + 1) % v.size()]);
	while (l < r) {
		int c = ((l + r) >> 1);
		int c_before = orientation(p, v[c], v[(c - 1) % v.size()]);
		int c_after = orientation(p, v[c], v[(c + 1) % v.size()]);
		int c_side = orientation(p, v[l], v[c]);
		if (c_before != RIGHT_TURN && c_after != RIGHT_TURN)
			return c;
		else if ((c_side == LEFT_TURN) && (l_after == RIGHT_TURN || l_before == l_after) || (c_side == RIGHT_TURN && c_before == RIGHT_TURN))
			r = c;
		else
			l = c + 1;
		l_before = -c_after;
		l_after = orientation(p, v[l], v[(l + 1) % v.size()]);
	}
	return l;
}

/*
	Returns the pair of integers representing the Hull # and the point in that Hull which is the extreme amongst all given Hull Points
	@param hulls: Vector containing the hull points for various hulls stored as individual vectors.
*/
pair<int, int> ConvexHull::extreme_hullpt_pair(vector<vector<pmp::vec3> >& hulls)
{
	int h = 0, p = 0;
	for (int i = 0; i < hulls.size(); ++i) {
		int min_index = 0, min_y = hulls[i][0][1];
		for (int j = 1; j < hulls[i].size(); ++j) {
			if (hulls[i][j][1] < min_y) {
				min_y = hulls[i][j][1];
				min_index = j;
			}
		}
		if (hulls[i][min_index][1] < hulls[h][p][1]) {
			h = i;
			p = min_index;
		}
	}
	return make_pair(h, p);
}

/*
	Returns the pair of integers representing the Hull # and the point in that Hull to which the point lpoint will be joined
	@param hulls: Vector containing the hull points for various hulls stored as individual vectors.
	@param lpoint: Pair of the Hull # and the leftmost extreme point contained in that hull, amongst all the obtained hulls
*/
pair<int, int> ConvexHull::next_hullpt_pair(vector<vector<pmp::vec3> >& hulls, pair<int, int> lpoint)
{
	pmp::vec3 p = hulls[lpoint.first][lpoint.second];
	pair<int, int> next = make_pair(lpoint.first, (lpoint.second + 1) % hulls[lpoint.first].size());
	for (int h = 0; h < hulls.size(); h++) {
		if (h != lpoint.first) {
			int s = tangent(hulls[h], p);
			pmp::vec3 q = hulls[next.first][next.second];
			pmp::vec3 r = hulls[h][s];
			int t = orientation(p, q, r);
			if (t == RIGHT_TURN || (t == COLLINEAR) && distance(p, r) > distance(p, q))
				next = make_pair(h, s);
		}
	}
	return next;
}

/*
	Constraint to find the outermost boundary of the points by checking if the points lie to the left otherwise adding the given point p
	Returns the Hull Points
	@param v: Vector of all the points
	@param p: New point p which will be checked to be in the Hull Points or not
*/
vector<pmp::vec3>ConvexHull::keep_left(vector<pmp::vec3>& v, pmp::vec3 p)
{
	while (v.size() > 1 && orientation(v[v.size() - 2], v[v.size() - 1], p) != LEFT_TURN)
		v.pop_back();
	if (!v.size() || v[v.size() - 1] != p)
		v.push_back(p);
	return v;
}

/*
	Graham Scan algorithm to find convex hull from the given set of points
	@param points: List of the given points in the cluster (as obtained by Chan's Algorithm grouping)
	Returns the Hull Points in a vector
*/
vector<pmp::vec3>ConvexHull::GrahamScan(vector<pmp::vec3>& points)
{
	if (points.size() <= 1)
		return points;
	auto bind_cmp = bind(&ConvexHull::compare, this, _1, _2);
	sort(points.begin(), points.end(), bind_cmp);
	vector<pmp::vec3> lower_hull;
	for (int i = 0; i < points.size(); ++i)
		lower_hull = keep_left(lower_hull, points[i]);
	reverse(points.begin(), points.end());
	vector<pmp::vec3> upper_hull;
	for (int i = 0; i < points.size(); ++i)
		upper_hull = keep_left(upper_hull, points[i]);
	for (int i = 1; i < upper_hull.size(); ++i)
		lower_hull.push_back(upper_hull[i]);
	return lower_hull;
}

/*
	Implementation of Chan's Algorithm to compute Convex Hull in O(nlogh) complexity
*/
vector<pmp::vec3>ConvexHull::chansalgorithm(vector<pmp::vec3> v)
{
	for (int t = 0; t < v.size(); ++t) 
	{
		for (int m = 1; m < (1 << (1 << t)); ++m) 
		{
			vector<vector<pmp::vec3> > hulls;
			for (int i = 0; i < v.size(); i = i + m) 
			{
				vector<pmp::vec3> chunk;
				if (v.begin() + i + m <= v.end())
					chunk.assign(v.begin() + i, v.begin() + i + m);
				else
					chunk.assign(v.begin() + i, v.end());
				hulls.push_back(GrahamScan(chunk));
			}
			cout << "\nM (Chunk Size): " << m << "\n";
			for (int i = 0; i < hulls.size(); ++i) {
				cout << "Convex Hull for Hull #" << i << " (Obtained using Graham Scan!!)\n";
				for (int j = 0; j < hulls[i].size(); ++j)
					cout << hulls[i][j] << " ";
				cout << "\n";
			}
			vector<pair<int, int> > hull;
			hull.push_back(extreme_hullpt_pair(hulls));
			for (int i = 0; i < m; ++i) {
				pair<int, int> p = next_hullpt_pair(hulls, hull[hull.size() - 1]);
				vector<pmp::vec3> output;
				if (p == hull[0]) {
					for (int j = 0; j < hull.size(); ++j) {
						output.push_back(hulls[hull[j].first][hull[j].second]);
					}
					return output;
				}
				hull.push_back(p);
			}
		}
	}
}/*

int main()
{
	int T = 0, x = 0, y = 0;
	cin >> T;
	if (T <= 0)
		return -1;
	pmp::vec3 points[T];
	for (int i = 0; i < T; ++i) {
		cin >> x >> y;
		points[i].x = x;
		points[i].y = y;
	}
	vector<pmp::vec3> v(points, points + T);
	vector<pmp::vec3> output = chansalgorithm(v);
	cout << "\n---------After Using Chan's Algorithm---------------\n";
	cout << "\n***************** CONVEX HULL **********************\n";
	for (int i = 0; i < output.size(); ++i)
		cout << output[i] << " ";
	cout << "\n";
	return 0;
}*/
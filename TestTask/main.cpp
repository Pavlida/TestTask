#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <unordered_set>
#include<algorithm>
#include <map>
#include <limits>

double R, a; //initial radius given and the side of a sqare put inside

double inf = std::numeric_limits<double>::infinity();

class Line;

class Point
{
public:

    Point(){}
    Point(double x, double y) : x(x), y(y) {}

    double x = 0;
    double y = 0;

    bool operator<(const Point& b) const
    {
        if (x == b.x)
                return y <= b.y;
        return x <= b.x;
    }

    static double dist(Point* from, Point* to)
    {
        double dy = to->y - from->y;
        double dx = to->x - from->x;

        return sqrt(pow(dy, 2) + pow(dx, 2));
    }

    Line* from = nullptr;
    Line* to = nullptr;
};

static const Point indicator = Point(inf, inf);


class Line //y = kx + b
{
public:

    Line(){}

    Line(Point* from, Point* to, bool supporting = false) : from(from), to(to)
    {
        double dy = to->y - from->y;
        double dx = to->x - from->x;

        len = sqrt(pow(dy, 2) + pow(dx, 2));

        slope = dy/dx;
        offset = to->y - slope * to->x;
        angle = std::atan(slope);
        if (angle < 0)
            angle += M_PI;

        if(!supporting)
        {
            from->from = this;
            to->to = this;
        }
    }

    Line(Point* from, double angle, double len) : from(from), angle(angle), len(len)
    {
        slope = std::tan(angle);
        to = new Point(from->x + len * std::cos(angle), from->y + len * std::sin(angle));
    }

    double len = 0;
    double slope = 0; //k
    double offset = 0; //b
    double angle = 0;


    //get angle of the lines that come from the point
    static double angleBetween(Line* lhs, Line* rhs) { return std::abs(rhs->angle - lhs->angle); } const

    Point moveAlong( double distance)
    {
        double ratio = len /distance;
        return Point((1-ratio) * from->x + ratio * to->x, (1-ratio) * from->y + ratio * to->y);
    }

    Point* intersection(const Line& other, double* distTransfer = nullptr)
    {
        Point* ofIntersection = new Point();
        ofIntersection->x = (other.offset - offset) / (slope - other.slope);
        ofIntersection->y = (offset * other.slope - other.offset * slope) /  (slope - other.slope);

        double dist = Point::dist(from, ofIntersection);
        double check = Point::dist(to, ofIntersection);

        if(dist + check > len) //outside the segment
        {
            delete ofIntersection;
            return nullptr;
        }

        if (distTransfer)
            *distTransfer = dist;

        return ofIntersection;
    }

    double toIntersection(const Line& other)
    {
        if (std::abs(slope) == std::abs(other.slope))
        {
            if(offset == other.offset)
            {
                if (other.from < other.to)
                {
                    if((other.from < from && from < other.to) || ((other.from < to && to < other.to))) //either point between ()
                        return inf; //they cross
                    return -1;
                }
                else
                {
                    if((other.to < from && from < other.from) || ((other.to < to && to < other.from)))
                        return inf; //they cross
                    return -1;
                }

            }
            else //parallel
                return -1;
        }

        double dist;
        Point* ofIntersection = intersection(other, &dist);

        if(ofIntersection) //there is a point of intersection
        {
            delete ofIntersection;
            return dist;
        }

        return -1;
    }

    double wiggleRoom()
    {
        if(angleBetween(from->to, this) < M_PI_4)
            return remainder(len, 2 * R);

        return remainder(len, a);
    }

    Point* getOrigin() const {return from;};
    Point* getDestination() const {return to;};

    double realAngle() {return std::atan2(to->y - from->y, to->x - from->x);}

    bool isOnLine(Point* p)
    {
        if(p->x * slope + offset == p->y)
            return true;
        return false;
    }

private:
    friend bool operator==(const Line& a, const Line& b);
    friend void finishCycle(std::vector<Line>& edges, Point* looper);
    Point* from = nullptr;
    Point* to = nullptr;
};

bool operator==(const Line& lhs, const Line& rhs) { return (lhs.from == rhs.from) && (lhs.to == rhs.to); }

//struct Comparator
//{
//    Comparator(std::vector<double> parameter) {}

//    bool operator()(const Line& lhs, const Line& rhs) const {return *lhs < *rhs;}

//private:

//};

void finishCycle(std::vector<Line>& edges, Point* looper)
{
    edges.push_back(Line(looper, edges[0].from));
}

std::vector<Line> Intersections(Line toCheck, const std::vector<Line>& allLines, std::vector<Line>& alined, bool toSort=true)
{
    std::vector<Line> result;
    std::vector<double> distances;

    for(int i =0; i < allLines.size(); ++i) //geting all intersecting lines
    {
        double dist = toCheck.toIntersection(allLines[i]);
        if(dist == -1)
            continue;
        else if (dist = inf)
        {
            alined.push_back(allLines[i]);
            continue;
        }
        result.push_back(allLines[i]);
        distances.push_back(dist);
    }

    if (toSort && !result.empty()) //sorting
    {
        std::multimap<double, Line> sorted;
        for(int i =0; i < result.size(); ++i)
            sorted.insert({distances[i], result[i]});
        result.clear();
        for(auto x : sorted)
            result.push_back(x.second);  //get data from sorted multimap
    }

    return result;
}

int main()
{
    std::vector<Point> centers;

    std::vector<Line> edges;
    std::set<Point> vertices;

    Point from(0,0); //read first
    vertices.insert(from);
    while(true) //reading
    {     
        //read;
        Point to(0, 0);
        edges.push_back(Line(&from, &to));
        vertices.insert(to);
        from = to;
    }
    finishCycle(edges, &from); //finish connecting the dots

    while(!vertices.empty())
    {
        auto covering = vertices.begin(); //since set is always sorted we get the left most point (1)
        Point center;
        bool pathToCenterClear = true;
        double checker = R;

        double angle = Line::angleBetween(covering->from, covering->to);

        double centerAngle = covering->from->angle;
        if(centerAngle > M_PI_2)
            centerAngle -= M_PI;

        if(angle > M_PI_4)
        {
            checker = a / 2;
            centerAngle -= M_PI_4;
            pathToCenterClear = false;
        }
        else if (covering->from->len < R)
            pathToCenterClear = false;

        Line toCenter(covering->from->getOrigin(), centerAngle, R);


        if(pathToCenterClear)
            center = *toCenter.getDestination();
        else
        {
            std::vector<Line> garbage;
            std::vector<Line> inter = Intersections(toCenter, edges, garbage, true);
            if(inter.empty())
                center = *toCenter.getDestination(); //pathtocenter clear
            else
                center = *toCenter.intersection(inter[0]);
        }

        //getting the diagonal point
        Point* diagonal = Line(&center, centerAngle, R).getDestination();

        Point* perpendicular = Line(&center, centerAngle - M_PI_2, R).getDestination();


        Line newEdgeLines[4];
        newEdgeLines[2] = Line(diagonal, perpendicular, true); //linePerpendicular to from always there
        //Line alongFrom(perpendicular.getDestination(), toCenter.getOrigin(), true); //will be neeeded later

        Point* starter = Line(&center, centerAngle - M_PI, R).getDestination();
        Point* parallel = nullptr;
        if (covering->from->len < checker)
        {
           parallel = Line(&center, centerAngle + M_PI_2, R).getDestination();
           newEdgeLines[0] = Line(starter, parallel);
        }
        else if (covering->from->len <  2 * checker)
        {
            newEdgeLines[1] = Line(covering->from->getDestination(), new Point(covering->from->moveAlong(2 * checker)) , true);
        }

        if (angle > M_PI_4)
            parallel = Line(&center, centerAngle + M_PI_2, R).getDestination();

        if(parallel)
             newEdgeLines[1] = Line(parallel, diagonal, true);

         newEdgeLines[3] = Line(perpendicular, toCenter.getOrigin(), true); //easier to just leave in always

         std::vector<Line> crossed;
         std::vector<Line> alined;

        for(int i = 0; i < 4; ++i)//get all intersections with existing edges
            if(newEdgeLines[i].getOrigin()) //origin non nullptr == line exists
                for(Line v : Intersections(newEdgeLines[i], edges, alined))
                    crossed.push_back(v);

        std::vector<Point> inside;
        bool in;

        while (!crossed.empty()) {

        }


        if(angle <= M_PI_2)
            vertices.erase(covering);

        centers.push_back(center);
    }

    for (Point center : centers)
        std::cout << "x: " << center.x << "y: " << center.y << std::endl;
    return 0;
}

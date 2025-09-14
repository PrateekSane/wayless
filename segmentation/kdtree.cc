#include <iostream>
#include <set>
using namespace std;

enum class Axis { x, y, z };

struct Point
{
    int x, y, z;
    Point(int x_, int y_, int z_): x(x_), y(y_), z(z_) { }
};

const int& get(const Point& p, Axis a) {
    switch (a) {
        case Axis::x: return p.x;
        case Axis::y: return p.y;
        case Axis::z: return p.z;
    }
    throw std::out_of_range("Invalid axis");
}

struct Node
{
    Point point;
    u_int point_id;
    Node* left;
    Node* right;
    /*
    data: dictionary of {x,y,z} coordinates
    point_id: index position of the point in the pcd data
    */
    Node(Point point_, u_int point_id_) : point(std::move(point_)), point_id(point_id_) { }
};


class KdTree
{
public:
    KdTree()
    {
        root = nullptr;
    }

    void insertPoints(vector<Point> points, bool display_output=false)
    {
        for (auto point: points)
        {
            root = buildKdTree(root, point, Axis::x);
        }

        if (display_output)
        {
            display_tree(root);
        }
    }

    /*
    Takes a point and inserts it into the tree 
    */
    Node* buildKdTree(Node* cur_node, Point point, Axis split_dir)
    {
        if (!cur_node)
        {
            Node* new_node = new Node(point, 0);
            return new_node;
        }

        int root_val = get(cur_node->point, split_dir);
        int check_val = get(point, split_dir);

        Axis next_dir = nextDirection(split_dir); 
        // decide whether to put on the left or right child

        if (check_val <= root_val)
        {
            cur_node->left = buildKdTree(cur_node->left, point, next_dir);
        }
        else 
        {
            cur_node->right = buildKdTree(cur_node->right, point, next_dir);
        }
        return cur_node;
    }

    void findAndShowElements(Point search_point)
    {
        
        searchElements(root, search_point, Axis::x, 2);
    }

    /*
    Finds closest point to the one passed in 
    */
    void searchElements(Node* root, Point search_point, Axis split, int threshold) const
    {
        if (!root)
        {
            return;
        }

        Point cur_point = root->point;
        bool x_thresh = search_point.x + threshold > cur_point.x && search_point.x - threshold < cur_point.x;
        bool y_thresh = search_point.y + threshold > cur_point.y && search_point.y - threshold < cur_point.y;
        bool z_thresh = search_point.z + threshold > cur_point.z && search_point.z - threshold < cur_point.z;
        if (x_thresh && y_thresh && z_thresh)
        {
            int dist = getDistance(cur_point, search_point);
            if (dist < threshold)
            {
                cout << "x: " << cur_point.x << " y: " << cur_point.y << " z: " << cur_point.z << endl;
            }
        }

        Axis next_split = nextDirection(split); 
        int root_val = get(root->point, split);
        int check_val = get(search_point, split);
        if (check_val <= root_val)
        {
            return searchElements(root->left, search_point, next_split, threshold);
        }
        else
        {

            return searchElements(root->right, search_point, next_split, threshold);
        }
        return;
    }

    void display_tree(Node* root) const 
    {
        if (!root)        
        {
            return;
        }

        cout << "x: " << root->point.x << " y: " << root->point.y << " z: " << root->point.z << endl;
        // cout << endl;
        display_tree(root->left);
        // cout << " " << endl;
        display_tree(root->right);
    }

    ~KdTree()
    {

    }

private:
    Node* root;   

    Axis nextDirection(Axis dir) const
    {
        if (dir == Axis::x)
        {
            return Axis::y;
        }
        else if (dir == Axis::y)
        {
            return Axis::z;
        }
        return Axis::z;
    }

    int getDistance(Point a, Point b) const 
    {
        return pow(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2), .5);
    }
};

int main()
{
    KdTree kdTree;
    Point point1(0, 0, 0);
    Point point2(0, 1, 0);
    Point point3(-1, 0, 1);
    Point point4(0, 2, -1);

    kdTree.insertPoints({point1, point3, point4, point2});
    
    kdTree.findAndShowElements(Point({0,0,0}));
    return 0;
}
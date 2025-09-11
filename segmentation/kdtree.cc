#include <iostream>
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

    /*
    Finds closest point to the one passed in 
    */
    u_int search_elements(Point points) const
    {
        return 0;
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

    Axis nextDirection(Axis dir)
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
};

int main()
{
    KdTree kdTree;
    Point point1(0, 0, 0);
    Point point2(0, 1, 0);
    Point point3(-1, 0, 1);
    Point point4(0, 2, -1);

    kdTree.insertPoints({point1, point3, point4, point2}, true);
    return 0;
}
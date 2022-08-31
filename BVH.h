#include "trace.H"

struct BVHNode
{
    AABB bound;
    uint32_t offset = 0;
    uint32_t numGeometry = 0;
};

class BVH
{
public:

    inline void buildBVH(std::vector<Surface*> objects)
    {
        _nodes.resize(objects.size() * 2);
        root = getRoot();

        
    }

    inline BVHNode* getRoot() { return _nodes.data(); }


private:

    void updateNode(uint32_t node)
    {
        BVHNode* pNode = &_nodes[node];


    }


    BVHNode* root = nullptr;
    std::vector<BVHNode> _nodes;
};

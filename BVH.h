#include "trace.H"
#include <algorithm>

struct BVHNode
{
    AABB bound;
    uint32_t offset = 0;
    uint32_t numGeometry = 0;
    bool isLeaf() const { return numGeometry > 0; }
};

class BVH
{
public:

    inline void buildBVH(std::vector<std::pair<Surface*, Fill>>* objects)
    {
        _nodes.resize(objects->size() * 2);
        _objects = objects;
        root = getRoot();
        root->numGeometry = objects->size();

        updateNode(rootIndex);
        buildImpl(rootIndex);
    }

    inline BVHNode* getRoot() { return _nodes.data(); }

    inline bool intersect(const Ray& r, double t0, double t1, HitRecord& hr)
    {
        return intersect(r, 0, t0, t1, hr);
    }

    inline bool intersect(const Ray& r, uint32_t node, double t0, double t1, HitRecord& hr)
    {
        BVHNode *pNode = &_nodes[node];

        if(!pNode->bound.intersect(r)) return false;
        bool isHit = false;
        if(pNode->isLeaf())
        {
            double minDistance = hr.t;
            HitRecord dummy;
            for(uint32_t i = 0; i < pNode->numGeometry; ++i)
            {
                auto obj = (*_objects)[pNode->offset + i];
                auto surface = obj.first;

                if(surface->intersect(r,t0, t1, dummy))
                {
                    isHit = true;
                    if(dummy.t < minDistance)
                    {
                        minDistance = dummy.t;
                        hr = dummy;
                        hr.f = obj.second;
                    }
                }
            }
        }
        else
        {
            isHit |= intersect(r, pNode->offset, t0, t1, hr);
            isHit |= intersect(r, pNode->offset + 1, t0, t1, hr);
        }
        return isHit;
    }

private:
    constexpr static size_t rootIndex = 0;

    inline void buildImpl(uint32_t node)
    {
        //if(_nodes[node].numGeometry <= 2) return;

        auto _extent = _nodes[node].bound.extent();
        uint32_t axis = 0;
        if(_extent.y() > _extent.x()) axis = 1;
        if(_extent.z() > _extent[axis]) axis = 2;

        auto mid = _nodes[node].bound.boundMin[axis] + _extent[axis] * 0.5;
        auto offset = _nodes[node].offset;
        auto numGeometry = _nodes[node].numGeometry;
        auto splitIte = std::partition(_objects->begin() + offset, _objects->begin() + offset + numGeometry, 
                       [&](auto& obj) { return obj.first->centroid()[axis] < mid;});

        size_t group1Size = splitIte - _objects->begin() - offset;
        if(group1Size == 0 || splitIte == _objects->begin() + offset + numGeometry) 
        {
            _nodes[node].offset = offset;
            return;
        }

        size_t leftIndex = _nodeCount++;
        size_t rightIndex = _nodeCount++;

        _nodes[leftIndex].offset = offset;
        _nodes[leftIndex].numGeometry = group1Size;

        _nodes[rightIndex].offset = offset + group1Size;
        _nodes[rightIndex].numGeometry = numGeometry - group1Size;

        // Mark this to indicate if current node is leaf node
        _nodes[node].numGeometry = 0;

        // Mark this to change current offset to node index
        _nodes[node].offset = leftIndex;

        updateNode(leftIndex);
        updateNode(rightIndex);

        buildImpl(leftIndex);
        buildImpl(rightIndex);
    }


    inline void updateNode(uint32_t node)
    {
        BVHNode* pNode = &_nodes[node];

        for(uint32_t i = pNode->offset; i < pNode->offset + pNode->numGeometry; ++i)
        {
            auto surface = (*_objects)[i].first;
            auto aabb = surface->buildAABB();
            pNode->bound.boundMax.maxSet(aabb.boundMax);
            pNode->bound.boundMin.minSet(aabb.boundMin);
        }
    }

    BVHNode* root = nullptr;
    std::vector<std::pair<Surface*, Fill>>* _objects;
    std::vector<BVHNode> _nodes;
    size_t _nodeCount = 1;
};

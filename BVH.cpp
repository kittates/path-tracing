#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

// 这里和Lab6多了一个area
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        // Bounds3 centroidBounds;
        // for (int i = 0; i < objects.size(); ++i)
        //     centroidBounds =
        //         Union(centroidBounds, objects[i]->getBounds().Centroid());
        // int dim = centroidBounds.maxExtent();
        // switch (dim) {
        // case 0:
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().x <
        //                f2->getBounds().Centroid().x;
        //     });
        //     break;
        // case 1:
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().y <
        //                f2->getBounds().Centroid().y;
        //     });
        //     break;
        // case 2:
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().z <
        //                f2->getBounds().Centroid().z;
        //     });
        //     break;
        // }

        // auto beginning = objects.begin();
        // auto middling = objects.begin() + (objects.size() / 2);
        // auto ending = objects.end();

        // auto leftshapes = std::vector<Object*>(beginning, middling);
        // auto rightshapes = std::vector<Object*>(middling, ending);

        // assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        // node->left = recursiveBuild(leftshapes);
        // node->right = recursiveBuild(rightshapes);

        // node->bounds = Union(node->left->bounds, node->right->bounds);
        // node->area = node->left->area + node->right->area;
        
        //-------replaced with SAH---------
        size_t n = objects.size();
        std::vector<Bounds3> leftBounds(n+3),rightBounds(n+3);
        Bounds3 b;
        for(int i=1;i<=n;i++) {
            if(i==1) b = objects[i-1]->getBounds();
            else b = Union(b,objects[i-1]->getBounds());
            leftBounds[i] = b;
        }
        for(int i=n;i>=1;i--) {
            if(i==n) b = objects[i-1]->getBounds();
            else b = Union(b,objects[i-1]->getBounds());
            rightBounds[i] = b;
        }
        int index = 0;
        double cost = std::numeric_limits<double>::infinity();
        double totalArea = bounds.SurfaceArea();
        for(int i=0;i<n-1;i++) {
            double leftArea = leftBounds[i+1].SurfaceArea();
            double rightArea = rightBounds[i+2].SurfaceArea();
            float curr_cost =  (leftArea/totalArea) * (double)(i+1) + (rightArea/totalArea) * (double)(n-i-1);
            if(curr_cost < cost) {
                cost = curr_cost;
                index=i+1;
            }
        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + index;
        auto ending = objects.end();
        
        auto leftshapes = std::vector<Object*>(beginning,middling);
        auto rightshapes = std::vector<Object*>(middling,ending);

        node->left = recursiveBuild(leftshapes);
        node->right =  recursiveBuild(rightshapes);
        node->object = nullptr;
        node->bounds = Union(node->left->bounds,node->right->bounds);
        //-------replaced with SAH---------
    }
    
    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    
    if(node==nullptr) return Intersection();

    std::array<int,3> dirIsNeg = {{ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0}};
    bool happen = node->bounds.IntersectP(ray,ray.direction_inv,dirIsNeg);
    if(!happen) return Intersection();  // 如果光线与该包围盒不相交，那么就直接返回空交点

    if(node->object) {  // 该node是leaf，判断条件也可以换为node->left==nullptr && node->right==nullptr
        return node->object->getIntersection(ray);  // inline Intersection Triangle::getIntersection(Ray ray)
    } 
    bool hasLeft = node->left != nullptr;
    bool hasRight = node->right != nullptr;

    std::vector<float> tL,tR;
    if(hasLeft) tL = node->left->bounds.IntersectP_d(ray,ray.direction_inv,dirIsNeg);
    if(hasRight) tR = node->right->bounds.IntersectP_d(ray,ray.direction_inv,dirIsNeg);
    BVHBuildNode *first = node->left,*second = node->right; 
    if(hasRight && (!hasLeft || tL[0]>tR[0])) {     // 尝试交换左右子树，让距离更近box的先遍历
        std::swap(first,second);
        std::swap(tL,tR);
    }
    Intersection isect_left = getIntersection(first,ray);
    if(!isect_left.happened) return getIntersection(second,ray);    // 剪枝 left无交点，return right
    else if(isect_left.happened && isect_left.distance < tR[0]) return isect_left;  // left有交点，且距离更近，return left
    else {                                                          // 其他情况，都需要判断left和right
        Intersection isect_right = getIntersection(second,ray);
        return isect_left.distance < isect_right.distance ? isect_left:isect_right;
    } 
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        // 调用该物体的Sample函数
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}
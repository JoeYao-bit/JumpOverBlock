//
// Created by yaozhuo on 8/1/25.
//

#ifndef JUMPOVERBLOCK_LAZY_THETA_STAR_H
#define JUMPOVERBLOCK_LAZY_THETA_STAR_H

#include "utility.hpp"
#include "pathfinding.hpp"
#include "../../algorithm/space_binary_tree/space_binary_tree.h"

namespace LazyThetaStar {


//The pathfinder is a general algorithm that can be used for mutliple purpose
//So it use adaptor
//This adaptor is for tile grid
    class MyAdaptor2D : public PathfinderAdaptor {
    public:


        MyAdaptor2D(freeNav::DimensionLength* dim, const freeNav::IS_OCCUPIED_FUNC<2>& isoc,
                    const freeNav::IS_LINE_COLLISION_FREE_FUNC<int, 2>& lsfr)
        : mMapSize(Vectori(dim[0],dim[1])), isoc_(isoc), lsfr_(lsfr), dim_(dim) {
        }

        virtual size_t getNodeCount() const override {
            return mMapSize.x * mMapSize.y;
        }

        //return the distance between two node
        virtual Cost distance(const NodeId n1, const NodeId n2) const override {
            return dist((Vectorf) idToPos(n1), (Vectorf) idToPos(n2));
        }

        //Return true if there is a direct path between n1 and n2
        //Totally not stole this code and did some heavy rewrite
        //The original code was way worse, trust me
        virtual bool lineOfSight(const NodeId n1, const NodeId n2) const override {
            return lsfr_(freeNav::IdToPointi<2>(n1, dim_), freeNav::IdToPointi<2>(n2, dim_));
        }

        //return a vector of all the neighbors ids and the cost to travel to them
        //In this adaptor we only need to check the four tileneibors and the cost is always 1
        virtual std::vector<std::pair<NodeId, Cost>> getNodeNeighbors(const NodeId id) const override {
            auto pos = idToPos(id);

            const Cost cost = 1;

            std::vector<std::pair<NodeId, Cost>> neighbors;

            //check if we are not on most left if not check if the tile to the left is traversable
            //if so then add it to the neighbor list with its cost(1 for all neighbors)
            if (pos.x != 0 && !isoc_(freeNav::Pointi<2>{pos.x - 1, pos.y}))
                neighbors.push_back({posToId({pos.x - 1, pos.y}), cost});

            if (pos.y != 0 && !isoc_(freeNav::Pointi<2>{pos.x, pos.y - 1}))
                neighbors.push_back({posToId({pos.x, pos.y - 1}), cost});

            if (pos.x != mMapSize.x - 1 && !isoc_(freeNav::Pointi<2>{pos.x + 1, pos.y}))
                neighbors.push_back({posToId({pos.x + 1, pos.y}), cost});

            if (pos.y != mMapSize.y - 1 && !isoc_(freeNav::Pointi<2>{pos.x, pos.y + 1}))
                neighbors.push_back({posToId({pos.x, pos.y + 1}), cost});

            return neighbors;
        }

        //custom function used to map tile to id
        NodeId posToId(const Vectori &pos) const {
            return pos.y * mMapSize.x + pos.x;
        }

        //custom function used to map id to tile
        Vectori idToPos(const NodeId id) const {
            return {id % mMapSize.x, id / mMapSize.x};
        }

    private:
        const Vectori mMapSize;

        freeNav::IS_OCCUPIED_FUNC<2> isoc_;

        freeNav::IS_LINE_COLLISION_FREE_FUNC<int, 2> lsfr_;

        freeNav::DimensionLength* dim_;

    };


}


#endif //JUMPOVERBLOCK_LAZY_THETA_STAR_H

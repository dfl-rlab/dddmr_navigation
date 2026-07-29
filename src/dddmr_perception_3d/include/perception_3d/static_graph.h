/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef PERCEPTION_3D_STATIC_GRAPH_H_
#define PERCEPTION_3D_STATIC_GRAPH_H_

/*For graph*/
#include <unordered_map>
#include <set>
#include <queue> 

typedef std::pair<unsigned int, float> edge_t;
typedef std::set<edge_t> edges_t;

//@ use unsigned int is because fast triangle return uint32
typedef std::unordered_map<unsigned int, edges_t> graph_t;

namespace perception_3d
{

class StaticGraph{

  public:
    StaticGraph();
    ~StaticGraph();
    void allocateGraph(unsigned long int m_size);
    void insertEdgeInNode(unsigned int node, edge_t& a_edge);
    void setPenality(unsigned int node, float weight);
    graph_t* getGraphPtr();
    edges_t getEdge(unsigned int node);
    float getNodeWeight(unsigned int node);
    void clear();
    unsigned long getSize(){return graph_ptr_->size();};
    unsigned long getNodeWeightSize(){return node_penality_.size();};

  private:
    graph_t* graph_ptr_;
    std::unordered_map<unsigned int, float> node_penality_;

};

}

#endif
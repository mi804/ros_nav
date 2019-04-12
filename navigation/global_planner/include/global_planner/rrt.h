/****************
2019.4.11
The homework of robots.
Add rrt(s) to the global_planner using the API of ROS
H.Z & MK.J
*****************/


#ifndef _RRT_H_
#define _RRT_H_
#include <global_planner/planner_core.h>
#include <global_planner/expander.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <ros/console.h>

namespace global_planner {
class Node{
    public:
        int x_;
        int y_;
        int index_;  // index in the map
        int father_;  //array index of father node
};
class Tree{
    public:
        Node *Nodes_;
        int length;
        int max_length;
        Tree(int ns):max_length(ns),length(0){
            Nodes_ = new Node [max_length];
            for(int i = 0; i < max_length; i++){
                Nodes_[i].x_ = Nodes_[i].y_ = Nodes_[i].index_ = Nodes_[i].father_ = -1;
            }
        }
        void Clear(){
            for(int i = 0; i < max_length; i++){
                Nodes_[i].x_ = Nodes_[i].y_ = Nodes_[i].index_ = Nodes_[i].father_ = -1;
            }
            length = 0;
        }
        int AddNode(int x, int y, int index, int father){
            if(length == max_length){
                ROS_ERROR("memory full");
                return -1;
            }
            Nodes_[length].x_ = x;
            Nodes_[length].y_ = y;
            Nodes_[length].index_ = index;
            Nodes_[length].father_ = father;
            length++;
            return 0;
        }
        int Find_Near(int x, int y){
            int min_index = 0;
            int min_dis = 1000000;
            for(int i = 0; i < length; i++){
                int cur_dis = abs(Nodes_[i].x_ - x) + abs(Nodes_[i].y_ - y);
                if( cur_dis <= min_dis ){
                    min_index = i;
                    min_dis = cur_dis;
                }
            }
            return min_index;
        }
        bool IfExist(int index){
            for(int i = 0; i < length; i++){
                if(Nodes_[i].index_ == index){
                    return true;
                }
            }
            return false;
        }
};


class RRTExpansion : public Expander 
{
  //RRTExpander need to be rewrite
    public:
        RRTExpansion(PotentialCalculator* p_calc, int nx, int ny);
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                float* potential);
        bool calculatePlan(std::vector<std::pair<float, float> >& path);
        //calculatePotentials need to be rewrite
        void Reclear();
    private:
        int MyRand(int low, int high);
        bool is_legal(int ind);
        bool is_not_col(int startx, int starty, int endx, int endy);
        int toward(int startx, int starty, int endx, int endy);
        int step_size;
        int arrive_offset;
        int start_x;
        int start_y;
        int end_x;
        int end_y;
        Tree T;
        unsigned char* costs;
        int end_tree_index;
};

} //end namespace global_planner

#endif
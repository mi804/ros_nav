/**********************************************************
 * 2019.4.11
 * The homework of robots.
 * Add rrt(s) to the global_planner using the API of ROS
 * H.Z & MK.J
 *  
 * 2019.4.19
 * Add more notes and write a report.
 * H.Z & MK.J
 **********************************************************/


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
        int index_;  //储存节点在地图中的索引
        int father_;  //储存父节点的索引
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
        /**
         * @brief Clear the former usage of the tree
         */
        void Clear(){
            for(int i = 0; i < max_length; i++){
                Nodes_[i].x_ = Nodes_[i].y_ = Nodes_[i].index_ = Nodes_[i].father_ = -1;
            }
            length = 0;
        }
        /**
         * @brief Add node to the tree
         * @param x The x coordinate of the node in the map
         * @param y The y coordinate of the node in the map
         * @param index The index of the node in the map
         * @param father The index of the node's father node in the map
         */
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
        /**
         * @brief Find the nearest node by Manhattan distance
         * @param x The x coordinate of the node in the map
         * @param y The y coordinate of the node in the map
         */
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
        /**
         * @brief Judge if the new node is in the map
         * @param index The index of the node in the map
         */
        bool IfExist(int index){
            for(int i = 0; i < length; i++){
                if(Nodes_[i].index_ == index){
                    return true;
                }
            }
            return false;
        }
        /**
         * @brief Judge Calculate the length of the path
         *        from current to start by 2-norm distance
         * @param n_T The current tree with a path
         * @param id The index of the current node
         * @param start_i The index of the start node, 
         *        can be changed to calculate part of the tree 
         */        
        int calculateCost(Tree n_T, int id, int start_i){
            Node *pre = &n_T.Nodes_[id];
            Node *cur = &n_T.Nodes_[id];
            int sum_of_cost = 0;
            while (cur->index_ != start_i) {
                cur = &n_T.Nodes_[cur->father_];
                sum_of_cost += abs(pre->x_ - cur->x_)*abs(pre->x_ - cur->x_)+
                abs(pre->y_ - cur->x_)*abs(pre->y_ - cur->x_);
                pre = cur;
            }
            return sum_of_cost;
        }
};


class RRTExpansion : public Expander 
{
  //RRTExpander need to be rewrite
    public:
        RRTExpansion(PotentialCalculator* p_calc, int nx, int ny);
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                float* potential,bool,bool,bool,bool);
        bool calculatePlan(std::vector<std::pair<float, float> >& path);
        bool improvePlan(std::vector<std::pair<float, float> >& path);
        //calculatePotentials need to be rewrite
        void Reclear();
    private:
        /**
         * @brief Generate a random integer from low to high
         * @param low The lower limit
         * @param high The higher limit
         */
        int MyRand(int low, int high);
        /**
         * @brief Judge if the node is in the map
         * @param ind The index of the node
         */        
        bool is_legal(int ind);
        /**
         * @brief Judge if the path will have a collision
         * @param startx,starty The coordinate of the start node
         * @param endx,endy The coordinate of the end node
         */        
        bool is_not_col(int startx, int starty, int endx, int endy);
        /**
         * @brief Generate a branch of the tree for one stepsize
         * @param startx,starty The coordinate of the start node
         * @param endx,endy The coordinate of the end node
         */                
        int toward(int startx, int starty, int endx, int endy);
        
        /* some params */
        int step_size;
        int arrive_offset;
        int start_x;
        int start_y;
        int end_x;
        int end_y;
        int guide_radius;
        int explore_radius;
        bool m_use_goal_guide;
        bool m_use_cut_bridge;
        float guide_rate;
        Tree T;
        Tree ET;

        unsigned char* costs;
        int end_tree_index;

        /* algorithm functions */
        bool single_rrt(unsigned char* costs_, double start_x1, double start_y1, double end_x1, double end_y1,
                                    int num_of_ptcs, float* potential);
        bool single_rrt_star(unsigned char* costs_, double start_x1, double start_y1, double end_x1, double end_y1,
                                    int num_of_ptcs, float* potential);
        bool connect_rrt(unsigned char* costs_, double start_x1, double start_y1, double end_x1, double end_y1,
                                    int num_of_ptcs, float* potential);
        bool cut_bridge();
};



} //end namespace global_planner
#endif
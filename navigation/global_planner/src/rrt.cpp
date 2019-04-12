/****************
2019.4.11
The homework of robots.
add rrt(s) to the global_planner using the API of ROS
H.Z & MK.J
*****************/

#include <global_planner/rrt.h>
#include <stdlib.h>
#include<costmap_2d/cost_values.h>
#include <math.h>
#include <iostream>
#include<vector>

namespace global_planner {

RRTExpansion::RRTExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys),T(ns_) {
    step_size = 10;
    arrive_offset = 20;
}
//Expander need to be rewrite

bool RRTExpansion::calculatePotentials(unsigned char* costs_, double start_x1, double start_y1, double end_x1, double end_y1,
                                        int num_of_ptcs, float* potential) {
    costs = costs_;
    start_x = start_x1;
    start_y = start_y1;
    end_x = end_x1;
    end_y = end_y1;
    ROS_DEBUG_STREAM("Hello " << "World");
    std::fill(potential, potential + ns_, POT_HIGH);  // about potential
    //num_of_ptcs = 5000
    int arr_pot = 10;

	//costs is the whole costmap
    int start_i = toIndex(start_x, start_y);
    potential[start_i] = arr_pot;
    T.AddNode(start_x, start_y, start_i, start_i);       //father is itself

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;
    int end_tree_index = 0;  // store the end node index
    while (true) {
        srand(time(0)); // get a random dot on map
        int px = MyRand(0, nx_);
        int py = MyRand(0, ny_);
        // find_nearest node
        int m_index = T.Find_Near(px, py);
        int cx = T.Nodes_[m_index].x_;
        int cy = T.Nodes_[m_index].y_;
        if(is_legal(toIndex(px, py))){  // point legal
            if(is_not_col(cx, cy, px, py)){  // no obstacle between them
                int new_index = toward(cx, cy, px, py); // toward one step;
                int ccx = new_index % nx_, ccy = new_index / nx_;
                T.AddNode(ccx, ccy, new_index, m_index);
                potential[new_index] = arr_pot;
                int dx = abs(ccx - end_x);
                int dy = abs(ccy - end_y);
                if(dx < arrive_offset && dy < arrive_offset){
                    T.AddNode(end_x, end_y, goal_i, T.length - 1);  // add endnode to tree
                    end_tree_index = T.length - 1;
                    potential[goal_i] = arr_pot;
                    break;            // arrive
                }
            }
        }
    }
    return true;
}

bool RRTExpansion::calculatePlan(std::vector<std::pair<float, float> >& path){
    Node *pre = &T.Nodes_[toIndex(end_x,end_y)];
    Node *cur = &T.Nodes_[toIndex(end_x,end_y)];
    std::pair<float, float> current;
    current.first = end_x;
    current.second = end_y;
    path.push_back(current);
    int c = 0;
    while (cur->index_ != toIndex(start_x, start_y)) {
        pre = cur;
        cur = &T.Nodes_[cur->father_];
        int sx = pre->x_, sy = pre->y_;
        int ex = cur->x_, ey = pre->y_;
        // 从pre点到cur点中间的每个点都需要加到path中
        int dx = abs(sx - ex), dy = abs(sy - ey);
        int xd = 1, yd = 1;
        if(dx != 0){
            xd = (ex - sx)/abs(sx - ex);
        }
        if(dy != 0){
            yd = (ey - sy) / abs(sy - ey);
        }
        int cx = sx; int cy = sy;
        if(cx != ex){
            while(true){
                cx += xd;
                current.first = cx;
                current.second = cy;
                path.push_back(current);
                c++;
                if(cx == ex){
                    break;
                }
            }
        }
        if(cy != ey){
            while(true){
                cy += yd;
                current.first = cx;
                current.second = cy;
                path.push_back(current);
                c++;
                if(cy == ey){
                    break;
                }
            }
        }
        if(c++>ns_*4){
            return false;
        }
        return true;
    }
}


int RRTExpansion::MyRand(int low, int high){
    int a = rand() % (high - low);
    a += low;
    return a;
}
bool RRTExpansion::is_legal(int ind){
    if(costs[ind]>=lethal_cost_ && !(unknown_ && costs[ind]==costmap_2d::NO_INFORMATION))
    {
        return false;
    }else{
        return true;
    }
}
bool RRTExpansion::is_not_col(int startx, int starty, int endx, int endy){
    int  cx = startx;
    int cy = starty;
    int i = 0;
    double xd = 1.0;
    double yd = 1.0;
    if(endx < startx) xd = -1.0;
    if(endy > starty) yd = -1.0;
    double dx = abs(endx - startx);
    double dy = abs(endy - starty);
    double theta = atan(dy/dx);
    double length = sqrt(dx * dx + dy * dy);
    int cycle = int(length / step_size);
    for(int i = 0; i < cycle; i++ ){
        int new_x = int (startx + xd * step_size * i * cos(theta));
        int new_y = int (starty + yd * step_size * i * sin(theta));
        if(!is_legal(toIndex(new_x,new_y))){
            return false;
        }
    }
    return true;
}


int RRTExpansion::toward(int startx, int starty, int endx, int endy){
    double xd = 1.0;
    double yd = 1.0;
    if(endx < startx) xd = -1.0;
    if(endy > starty) yd = -1.0;
    double dx = abs(endx - startx);
    double dy = abs(endy - starty);
    double theta = atan(dy/dx);
    int new_x = int (startx + xd * step_size * cos(theta));
    int new_y = int (starty + yd * step_size * sin(theta));
    return toIndex(new_x,new_y);
}
} //end namespace global_planner

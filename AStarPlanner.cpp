#include "AStarPlanner.h"
#include <queue>
#include <unordered_map>
#include <algorithm> // reverse
#include <iostream>

ostream& operator<<(ostream& os, const Path& path)
{
    for (auto loc : path) {
        os << loc << " ";
    }
    return os;
}

Path AStarPlanner::make_path(const AStarNode* goal_node) const {
    Path path;
    const AStarNode* curr = goal_node;
    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
    return path;
}

Path AStarPlanner::find_path(int agent_id, const list<Constraint>& constraints) {

    //find the start and goal location of the agent
    int start_location = ins.start_locations[agent_id];
    int goal_location = ins.goal_locations[agent_id];

    // Open list
    priority_queue<AStarNode*, vector<AStarNode*>, CompareAStarNode> open;
    
    // Unordered map is an associative container that contains key-value pairs with unique keys.
    // The following unordered map is used for duplicate detection, where the key is the location of the node.
    
    //map a pair that denotes the location and time step with an Astar node
    unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes;

    //find the heuristic value using manhattan distance
    int h = ins.get_Manhattan_distance(start_location, goal_location); // h value for the root node

    //edited the root node so that it has timestep = 0
    AStarNode* root = new AStarNode(start_location, 0, h, nullptr, 0); 

    //push root node to the open list
    open.push(root);

    //create a path
    Path path;

    //make an add neighbor boolean
    bool add_neighbor = true;

    //run A* search algorithm
    while (!open.empty()) {
        
        //explore curr node on the top of the queue
        AStarNode* curr = open.top();

        // cout<<"looking for a path at t = "<<curr->timestep;
        if(curr->timestep > (ins.map_size())){
            break;
        }

        //pop the curr from the queue
        open.pop();

        //goal test
        if (curr->location == goal_location) {
            path = make_path(curr);
            break;
        }

        // generate child nodes
        for (int next_location : ins.get_adjacent_locations(curr->location)) {
            
            //generate an iterator for the next node to see if the location has been visited
            auto it = all_nodes.find(make_pair(next_location,curr->timestep+1));
            
            //if the location has not been visited
            if (it == all_nodes.end()) {

                //make the next g and next h
                int next_g = curr->g + 1;
                int next_h = ins.get_Manhattan_distance(next_location, goal_location);

                //create the next location with a time step
                AStarNode* next = new AStarNode(next_location, next_g, next_h, curr, curr->timestep+1);
                
                //neightbor to be added unless proven guilty
                //check if next violates any constraints
                add_neighbor = check_constraints(agent_id, next, constraints);
               
                //add the next location to the open list
                if(add_neighbor){
                    open.push(next);
                }

                //add one to the child node
                all_nodes[make_pair(next_location, curr->timestep+1)] = next;
            }   

            // Note that if the location has been visited before,
            // next_g + next_h must be greater than or equal to the f value of the existing node,
            // because we are searching on a 4-neighbor grid with uniform-cost edges.
            // So we don't need to update the existing node.
        }

        //add the wait node option to the possible neighbors
        int wait_g = curr->g + 1;
        int wait_h = ins.get_Manhattan_distance(curr->location, goal_location);
        AStarNode* wait = new AStarNode(curr->location, wait_g, wait_h, curr, curr->timestep+1);

        //neightbor to be added unless it violates a constraint
        add_neighbor = check_constraints(agent_id, wait, constraints);

         //add the next location to the open list
        if(add_neighbor){
            open.push(wait);
            }
    }

    // release memory
    for (auto n : all_nodes)
        delete n.second;

    return path;
}

bool AStarPlanner::check_constraints(int agent_id, AStarNode* next, const list<Constraint>& _constraints){

    //if this stays true, the neighbor will be added to the queue
   bool add_neighbor = true; 

   //go through all of the constraints
   //to see if a neighbor is valid
   for(auto c : _constraints){

            //check vertex constraints
            if(get<4>(c) == VERTEX){
                if(get<0>(c)==agent_id && get<1>(c)==next->location && get<3>(c)==next->timestep){
                    add_neighbor = false;
                        break;
                        }
                    }
                //else check edge constraint numbers
                else if(get<4>(c)==EDGE){  
                        if(get<0>(c)==agent_id && get<1>(c)==next->parent->location && get<2>(c)==next->location && get<3>(c)==next->timestep){
                            add_neighbor = false;
                            break;
                        }
                }

                //added a goal which is a constraint that does not allow things to cross if
                //its at or after the current time step
                else if(get<4>(c) == GOAL){
                    if(get<0>(c)==agent_id && get<1>(c)==next->location && get<3>(c)<=next->timestep){
                        add_neighbor = false;
                        break;
                        }
                }
    }
                
    //return a bool whether or not a constraint was violated or not by the neighbor
    return add_neighbor;
}
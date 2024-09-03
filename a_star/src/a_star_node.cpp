#include <a_star_planner/a_star_node.hpp>
#include <a_star_planner/a_star.hpp>

namespace planning {
void AStarNode::generate_path(){
    AStar::Generator generator;
    generator.setWorldSize({300, 300});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    auto path = generator.findPath({0, 0}, {34, 27});

    for (auto& coordinate: path){
        std::cout << "[x: " << coordinate.x << ", y: " << coordinate.y << "]\n";
    }   
}
}
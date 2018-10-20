#include "agent.h"
#include "swarm_tools.h"

namespace agent{
swarm_tools::Point2D AgentState::position() const{
    swarm_tools::Point2D p = {this->x, this->y};
    return p;
    }
}
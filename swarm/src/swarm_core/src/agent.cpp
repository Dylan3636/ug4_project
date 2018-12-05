#include <assert.h>
#include "agent.h"

namespace agent{

    void USVAgent::copy(const USVAgent &usv){
        BaseAgent::copy(usv);
        set_current_assignment(usv.get_current_assignment());
    }
    AgentAssignment USVAgent::get_current_assignment() const{
        return this->current_assignment;
    }
    void USVAgent::set_current_assignment(AgentAssignment assignment){
        this->current_assignment=assignment;
    }
}
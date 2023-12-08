//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "time_guardian.h"

using namespace std;
using namespace goc;

namespace tspd {

TimeGuardian time_guardian;    
    
TimeGuardian::TimeGuardian()
  : rolex(true) 
{}
    
void TimeGuardian::SetTimeLimit(const goc::Duration& limit) 
{
    time_limit = limit;
}
    
void TimeGuardian::FailIfTimeLimit() const
{
    if(rolex.Peek() > time_limit) 
        throw TimeLimitExceeded();
}

Duration TimeGuardian::Now() const
{
    return rolex.Peek();
}
    
    
}

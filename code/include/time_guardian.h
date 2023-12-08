//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//
 
#include <exception>
#include "goc/goc.h"

#ifndef TSPD_EXCEPTION_H
#define TSPD_EXCEPTION_H

namespace tspd {

/*
 * Exception to control the time limit.
 */
struct TimeLimitExceeded : public std::exception {
    inline const char * what () const noexcept{
        return "Time Limit Exceeded";
    }
};

/*
 * A simple class to keep a global watch that allows us to fail in a centralized manner
 * when the time limit is reached.
 */
class TimeGuardian {
public:
    TimeGuardian();

    void SetTimeLimit(const goc::Duration& dur);
    void FailIfTimeLimit() const;
    goc::Duration Now() const;
    
private:
    goc::Stopwatch rolex;
    goc::Duration time_limit;
};

extern TimeGuardian time_guardian;

} //namespace tspd
#endif //TSPD_EXCEPTION_H

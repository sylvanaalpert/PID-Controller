#include "PID.h"
#include <limits>
#include <numeric>
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() :
    m_pError(0.0),
    m_iError(0.0),
    m_dError(0.0)
{
    m_bestErr = std::numeric_limits<double>::max();
    m_dp = {1.0, 1.0, 1.0};

}

PID::~PID() {}

void PID::init(double inKp, double inKi, double inKd, bool inTwiddle, double inTol) 
{
    m_pError = 0.0;
    m_iError = 0.0;
    m_dError = 0.0;

    m_Kp = inKp;
    m_Ki = inKi;
    m_Kd = inKd;

    m_p = {m_Kp, m_Ki, m_Kd};
    m_dp = {0.2*m_Kp, 0.2*m_Ki, 0.2*m_Kd};
    m_totalErr = 0.0;

    m_twiddle = inTwiddle;
    m_init = false;
    m_tol = inTol;
}

void PID::reinitializeForTwiddle()
{
    m_pError = 0.0;
    m_iError = 0.0;
    m_dError = 0.0;
    m_init = false;

    m_Kp = m_p[0];
    m_Ki = m_p[1];
    m_Kd = m_p[2];
    m_totalErr = 0.0;
}

void PID::updateError(double cte) 
{
    if (!m_init)
    {
        m_pError = cte;
        m_init = true;
    }
    
    m_dError = cte - m_pError;
    m_pError = cte;
    m_iError += cte;

    twiddle(cte);

}

double PID::getNewValue() const
{
    return -m_Kp * m_pError - m_Kd * m_dError - m_Ki * m_iError;
}


void PID::twiddle(double cte)
{
    if (m_twiddle)
    {

        uint32_t step = m_step % (m_evalSteps + m_settleSteps);

        if (step == 0 && (m_step != 0))
        {
            m_totalErr /= m_evalSteps;

            std::cout << "Param index: " << m_pIdx << std::endl;
            std::cout << "p: [" << m_p[0] << ", " << m_p[1] << ", " << m_p[2] << "] ";
            std::cout << "   dp: [" << m_dp[0] << ", " << m_dp[1] << ", " << m_dp[2] << "] " << std::endl;
            std::cout << "Total error: " << m_totalErr << std::endl;
            std::cout << "Best error:  " << m_bestErr << std::endl;

            // Check error and adjust params
            if (m_totalErr < m_bestErr)
            {
                m_bestErr = m_totalErr;
                m_dp[m_pIdx] *= 1.1;
                m_subtract = false;

                std::cout << "Parameter optimized - Best params: " << std::endl;
                std::cout << "  Kp: " << m_p[0] << std::endl;
                std::cout << "  Ki: " << m_p[1] << std::endl;
                std::cout << "  Kd: " << m_p[2] << std::endl;
            }
            else if (m_subtract)
            {
                m_p[m_pIdx] += m_dp[m_pIdx];
                m_dp[m_pIdx] *= 0.9;
                m_subtract = false;

            }
            else
            {
                m_p[m_pIdx] -= 2 * m_dp[m_pIdx];
                m_step = m_pIdx * (m_evalSteps + m_settleSteps) + 1;
                m_subtract = true;
                reinitializeForTwiddle();
                return;
            }


            if (m_pIdx == m_p.size() - 1)
            {
                // New iteration, check tolerance
                double s = std::accumulate(m_dp.begin(), m_dp.end(), 0.0);
                if (s < m_tol)
                {
                    reinitializeForTwiddle();
                    m_twiddle = false;

                    std::cout << "Twiddle finished - Best params: " << std::endl;
                    std::cout << "  Kp: " << m_Kp << std::endl;
                    std::cout << "  Ki: " << m_Ki << std::endl;
                    std::cout << "  Kd: " << m_Kd << std::endl;
                    return;
                }
                m_step = 0;
            }

            m_pIdx = (m_pIdx + 1) % m_p.size();
            m_p[m_pIdx] += m_dp[m_pIdx];
            reinitializeForTwiddle();
            
        }
        else if(step > m_settleSteps)
        {
            // Evaluate error
            m_totalErr += std::pow(cte, 2);
        }
        

        ++m_step;
        
    }
}

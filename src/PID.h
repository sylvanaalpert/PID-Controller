#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
   

   /*
   * Constructor
   */
   PID();

   /*
   * Destructor.
   */
   virtual ~PID();

   /*
   * Initialize PID.
   */
   void init(double inKp, double inKi, double inKd, bool inTwiddle = false, double inTol = 0.2);

   /*
   * Update the PID error variables given cross track error.
   */
   void updateError(double cte);

   double getNewValue() const;

private: 

   /*
   * Errors
   */
   double m_pError;
   double m_iError;
   double m_dError;

   /*
   * Coefficients
   */ 
   double m_Kp;
   double m_Ki;
   double m_Kd;

   bool m_init = false;
   bool m_twiddle = false;

   /* 
   * Twiddle params
   */
   std::vector<double> m_p;
   std::vector<double> m_dp;
   double m_tol = 0.2;
   double m_bestErr;
   double m_totalErr;
   uint32_t m_settleSteps = 100;
   uint32_t m_evalSteps = 3000;
   uint32_t m_step = 0;
   uint32_t m_pIdx = 0;
   bool     m_subtract = false;


   void twiddle(double cte);

   void reinitializeForTwiddle();


};

#endif /* PID_H */

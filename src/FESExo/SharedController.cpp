#include <Mahi/Util.hpp>
#include <SharedController.hpp>

SharedController::SharedController(double fes_share_amt, double exo_share_amt):
        m_fes_share_amt(fes_share_amt),
        m_exo_share_amt(exo_share_amt)
        {
            m_valid = verify_share_amts();
        }

SharedController::~SharedController()
{
}

bool SharedController::verify_share_amts(){
    if (m_fes_share_amt < 0.0 || m_exo_share_amt < 0.0){
        LOG(Error) << "Share amount cannot have a value less than 0.0";
        return false;
    }
    if (m_fes_share_amt + m_exo_share_amt > 1.0){
        LOG(Error) << "Combined share amounts cannot have a value greater than 1.0";
        return false;
    }
    return true;
}

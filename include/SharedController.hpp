class SharedController
{
private:
    bool verify_share_amts();
    bool m_valid = false;
    double m_fes_share_amt = 0;
    double m_exo_share_amt = 0;
public:
    SharedController(double fes_share_amt = 0.5, double exo_share_amt = 0.5);
    ~SharedController();
};

struct SharedTorques
{
    double exo_torque = 0;
    double fes_torque = 0;
};

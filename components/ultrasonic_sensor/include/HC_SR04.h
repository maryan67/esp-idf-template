


class HC_SR04 
{
    private:
    gpio_num_t m_trigger_e;
    gpio_num_t m_echo_e;
    float m_distance; 


    public:
    HC_SR04();
    readDistance();
    float getDistance()
    {
        return m_distance;
    }

}
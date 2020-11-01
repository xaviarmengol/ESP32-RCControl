class EnableSystem
{
private:
    bool stateChangingEnable = false;
    unsigned long initTimeInStateChangingEnable=0;
    bool enableRemote = true;

public:
    EnableSystem(/* args */);
    ~EnableSystem();
};

EnableSystem::EnableSystem(/* args */)
{
}

EnableSystem::~EnableSystem()
{
}

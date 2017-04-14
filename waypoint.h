class  waypoint
{
    
public:
    waypoint(float pLong = 0, float pLat = 0)
    {
        fLong = pLong;
        fLat = pLat;
    }
    
    float getLat(void) {return fLat;}
    float getLong(void) {return fLong;}
    
private:
    float fLong, fLat;
    
    
};  // waypoint


// usage as array:
//waypoint myWaypoints[4] = {waypoint(1,2), waypoint(2,3), waypoint(4,5), waypoint() };


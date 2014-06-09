//
//  Loads physics sprites created with http://www.PhysicsEditor.de
//
//  Generic Shape Cache for Chipmunk
//

#include "GCpShapeCache-x.h"
#include "chipmunk/CCPhysicsHelper_chipmunk.h"

using namespace gcp;

std::vector<std::string> splitStr(std::string str, std::string delim)
{
    std::vector<std::string> result;
    int cutAt;
    while( (cutAt = str.find_first_of(delim)) != str.npos ) {
        if(cutAt > 0) {
            result.push_back(str.substr(0, cutAt));
        }
        str = str.substr(cutAt + 1);
    }
    if(str.length() > 0) {
        result.push_back(str);
    }
    return result;
}

std::string replaceStr(std::string str, const std::string from, const std::string to)
{
    std::string::size_type pos = 0;
    while(pos = str.find(from, pos), pos != std::string::npos) {
        str.replace(pos, from.length(), to);
        pos += to.length();
    }
    return str;
}

Point GCpShapeCache::getPointFromString(std::string str)
{
    std::string theStr = str;
    
    theStr = replaceStr(theStr, "{ ", "");
    theStr = replaceStr(theStr, " }", "");
    
    auto array = splitStr(theStr, ",");
    if (array.size() <= 1) {
        return Point::ZERO;
    }
    return Point(Value(array.at(0)).asInt(), Value(array.at(1)).asInt());
}

cpFloat area(Point *vertices, int numVertices)
{
    cpFloat area = 0.0f;
    int r = (numVertices - 1);
    
    area += vertices[0].x * vertices[r].y - vertices[r].x * vertices[0].y;
    
    for (int i = 0; i < numVertices - 1; ++i) {
        area += vertices[r - i].x * vertices[r - (i + 1)].y - vertices[r - (i + 1)].x *
                vertices[r - i].y;
    }
    area *= 0.5f;
    return area;
}

static GCpShapeCache *s_pGCpShapeCache = nullptr;

enum FixtureType {
    FIXTURE_POLYGON,
    FIXTURE_CIRCLE
};

class Polygon
{
public:
    Point *vertices;
    int numVertices;
    cpFloat area;
    cpFloat mass;
    cpFloat momentum;
};

class FixtureData
{
public:
    FixtureData();
    ~FixtureData();
    
    FixtureType fixtureType;
    
    cpFloat mass;
    cpFloat elasticity;
    cpFloat friction;
    
    cpVect surfaceVelocity;
    
    cpCollisionType collisionType;
    cpGroup group;
    cpLayers layers;
    
    cpFloat area;
    cpFloat momentum;
    
    bool isSensor;
    
    Point center;
    cpFloat radius;
    
    std::vector<Polygon *> polygons;
    
};

FixtureData::FixtureData()
{
    polygons = std::vector<Polygon *>();
}

FixtureData::~FixtureData()
{
}

class BodyDef
{
public:
    BodyDef();
    ~BodyDef();
    
    Point anchorPoint;
    std::vector<FixtureData *> fixtures;
    float mass;
    float momentum;
};

BodyDef::BodyDef()
{
    fixtures = std::vector<FixtureData *>();
}

BodyDef::~BodyDef()
{
}

GCpShapeCache *GCpShapeCache::getInstance()
{
    if (s_pGCpShapeCache == nullptr) {
        s_pGCpShapeCache = new GCpShapeCache();
    }
    return s_pGCpShapeCache;
}

GCpShapeCache::GCpShapeCache()
{
    bodyDefs = std::map<std::string, BodyDef *>();
}

GCpShapeCache::~GCpShapeCache()
{
}

PhysicsBody *GCpShapeCache::createPhysicsBodyWithName(std::string name)
{
    BodyDef *bd = bodyDefs[name];
    CCASSERT(bd != 0, "Body not found");
    if (!bd) {
        return nullptr;
    }
    
    PhysicsBody *pBody = PhysicsBody::create(bd->mass, bd->momentum);
    for (auto& fd : bd->fixtures) {
        if (fd->fixtureType == FIXTURE_CIRCLE) {
            PhysicsMaterial m = PhysicsMaterial();
            m.friction = fd->friction;
            m.restitution = fd->elasticity;
            PhysicsShapeCircle *shape = PhysicsShapeCircle::create(fd->radius, m, fd->center);
            shape->setGroup(fd->group);
            
            pBody->addShape(shape);
        } else {
            for (auto& p : fd->polygons) {
                PhysicsMaterial m = PhysicsMaterial();
                m.friction = fd->friction;
                m.restitution = fd->elasticity;
            
                PhysicsShapePolygon *shape = PhysicsShapePolygon::create(p->vertices, p->numVertices, m);
                shape->setGroup(fd->group);
                
                pBody->addShape(shape);
            }
        }
    }
    
    return pBody;
}


cpVect* points2cpvs(Point* points, int num)
{
    cpVect *vertices = (cpVect *)malloc(sizeof(cpVect) * num);
    
    for (int i = 0; i < num; i++) {
        vertices[i] = PhysicsHelper::point2cpv(points[i]);
    }
    return vertices;
}

bool GCpShapeCache::addShapesWithFile(std::string plist)
{
    std::string path = FileUtils::getInstance()->fullPathForFilename(plist);
    ValueMap map = FileUtils::getInstance()->getValueMapFromFile(path.c_str());
    
    CCASSERT( !map.empty(), "Physics: file not found");
    
    ValueMap metadataMap = map["metadata"].asValueMap();
    int format = metadataMap["format"].asInt();
    if (format != 1) {
        return false;
    }
    
    ValueMap bodyMap = map["bodies"].asValueMap();
    for (auto& kv : bodyMap) {
        std::string bodyName = kv.first;
        ValueMap bodyDataMap = kv.second.asValueMap();
        
        BodyDef *bodyDef = new BodyDef();
        bodyDefs[bodyName] = bodyDef;
        
        bodyDef->anchorPoint = getPointFromString(bodyDataMap["anchorpoint"].asString());
        ValueVector fixtureList = bodyDataMap["fixtures"].asValueVector();
        
        float totalMass = 0.1f;
        cpFloat totalBodyMomentum = 0.1f;
        
        for (auto& kv2 : fixtureList) {
            FixtureData *fd = new FixtureData();
            if (!fd) {
                return false;
            }
            
            bodyDef->fixtures.push_back(fd);
            
            ValueMap fixtureData = kv2.asValueMap();
            fd->friction = fixtureData["friction"].asFloat();
            fd->elasticity = fixtureData["elasticity"].asFloat();
            fd->mass = fixtureData["mass"].asFloat();
            
            Point s = getPointFromString(fixtureData["surface_velocity"].asString());
            fd->surfaceVelocity = PhysicsHelper::point2cpv(s);
            
            fd->layers = fixtureData["layers"].asInt();
            fd->group = fixtureData["group"].asInt();
            fd->collisionType = fixtureData["collision_type"].asInt();
            fd->isSensor = fixtureData["fixtureData"].asBool();
            
            std::string fixtureType = fixtureData["fixture_type"].asString();
            cpFloat totalArea = 0.0f;
            
            totalMass += fd->mass;
            
            if (!fixtureType.compare("POLYGON")) {
                ValueVector polygonsArray = fixtureData["polygons"].asValueVector();
                fd->fixtureType = FIXTURE_POLYGON;
                
                for (auto& kv3 : polygonsArray) {
                    Polygon *poly = new Polygon();
                    if (!poly) {
                        return false;
                    }
                    
                    fd->polygons.push_back(poly);
                    
                    ValueVector polygonArray = kv3.asValueVector();
                    poly->numVertices = polygonArray.size();
                    
                    Point *vertices = poly->vertices = (Point *)malloc(sizeof(Point) * poly->numVertices);
                    if (!vertices) {
                        return false;
                    }
                    
                    int vindex = 0;
                    for (auto& kv4 : polygonArray) {
                        std::string pointString = kv4.asString();
                        Point offset = getPointFromString(pointString);
                        vertices[vindex] = offset;
                        vindex++;
                    }

                    
                    
                    poly->area = area(vertices, poly->numVertices);
                    
                    totalArea += poly->area;
                }
            } else if (!fixtureType.compare("CIRCLE")) {
                fd->fixtureType = FIXTURE_CIRCLE;
                
                ValueMap circleData = fixtureData["circle"].asValueMap();
                
                fd->radius = circleData["radius"].asFloat();
                Point p = getPointFromString(circleData["position"].asString());
                fd->center = p;
                totalArea += 3.1415927 * fd->radius * fd->radius;
            } else {
                CCASSERT(0, "Physics: unknown type");
            }
            
            fd->area = totalArea;
            
            cpFloat totalFixtureMomentum = 0.1f;
            
            if (totalArea) {
                if (fd->fixtureType == FIXTURE_CIRCLE) {
                    totalFixtureMomentum += cpMomentForCircle(fd->mass, fd->radius, fd->radius, PhysicsHelper::point2cpv(fd->center));
                } else {
                    for (auto& p : fd->polygons) {
                        p->mass = (p->area * fd->mass) / fd->area;
                        
                        cpVect* v = points2cpvs(p->vertices, p->numVertices);
                        
                        p->momentum = cpMomentForPoly(p->mass, p->numVertices, v, PhysicsHelper::point2cpv(Point::ZERO));
                        
                        totalFixtureMomentum += p->momentum;
                    }
                }
            }
            
            fd->momentum = totalFixtureMomentum;
            totalBodyMomentum = totalFixtureMomentum;
            
        }
        
        bodyDef->mass = totalMass;
        bodyDef->momentum = totalBodyMomentum;
    }
    
    return true;
}

Point GCpShapeCache::getAnchorPointForName(std::string name)
{
    BodyDef *bd = bodyDefs[name];
    CCASSERT(bd != 0, "Body not found");
    if (!bd) {
        return Point(0.5, 0.5);
    }
    return bd->anchorPoint;
}

std::vector<std::string> GCpShapeCache::getBodiesName()
{
    std::vector<std::string> bodyWithName;
    for (auto& b : bodyDefs) {
        std::string bodyName = b.first;
        bodyWithName.push_back(bodyName);
    }
    return bodyWithName;
}

std::vector<PhysicsShape *> GCpShapeCache::getShapesOfBodyWithName(std::string name)
{
    BodyDef *bd = bodyDefs[name];
    CCASSERT(bd != 0, "Body not found");
    if (!bd) {
        return std::vector<PhysicsShape *>();
    }
    
    std::vector<PhysicsShape *> shapes;
    
    for (auto& fd : bd->fixtures) {
        if (fd->fixtureType == FIXTURE_CIRCLE) {
            PhysicsMaterial m = PhysicsMaterial();
            m.friction = fd->friction;
            m.restitution = fd->elasticity;
            
            PhysicsShapeCircle *shape = PhysicsShapeCircle::create(fd->radius, m, fd->center);
            shape->setGroup(fd->group);
            
            shapes.push_back(shape);
        } else {
            for (auto& p : fd->polygons) {
                PhysicsMaterial m = PhysicsMaterial();
                m.friction = fd->friction;
                m.restitution = fd->elasticity;
            
                PhysicsShapePolygon *shape = PhysicsShapePolygon::create(p->vertices, p->numVertices, m);
                shape->setGroup(fd->group);
                
                shapes.push_back(shape);
            }
        }
    }
    
    return shapes;
}

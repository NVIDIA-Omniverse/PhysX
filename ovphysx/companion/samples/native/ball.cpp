// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-GEOMETRY-001
// @covers AC-1 AC-2 AC-3
// @implements REQ-COMP-NATIVE-SCHEMA-001
// @covers AC-3
#include <ovphysx_companion/native_sdk.h>
#include <geomutils/PxContactBuffer.h>
#include <common/PxRenderOutput.h>
#include <atomic>
#include <new>
using namespace physx;
namespace sdk=ovphysx::companion::native;
namespace {
std::atomic<uint32_t> creates{0},releases{0},contacts{0},starts{0},stops{0},rays{0},overlaps{0},sweeps{0},draws{0};
struct Context {sdk::HostApi host;};
struct Ball {float radius; PxCustomGeometry::Type type; Context* context;};
void* create(const sdk::ObjectContext& input,const PxCustomGeometry::Type& type,void* context) noexcept {
    if(input.parameterCount!=1 || input.parameters[0].scalar<=0) return nullptr;
    Ball * ball=new(std::nothrow) Ball{input.parameters[0].scalar,type,static_cast<Context*>(context)};
    if(ball) ++creates;
    return ball;
}
void release(void* object,void*) noexcept {++releases;delete static_cast<Ball*>(object);}
PxBounds3 bounds(const PxGeometry&,void* object) {
#ifdef COMPANION_TEST_BAD_BOUNDS
    return PxBounds3(PxVec3(1),PxVec3(-1));
#else
    const float r=static_cast<Ball*>(object)->radius;return PxBounds3(PxVec3(-r),PxVec3(r));
#endif
}
// This pedagogical sample implements plane/sphere/its own ball contacts. Other
// pairs explicitly produce no contacts; implement those before using it generally.
bool generate(const PxGeometry&,const PxGeometry& other,const PxTransform& a,const PxTransform& b,
              float distance,float,float,PxContactBuffer& out,void* object,void* otherObject) {
    const Ball & ball=*static_cast<Ball*>(object);
    PxVec3 normal;float separation;PxVec3 point;
    if(other.getType()==PxGeometryType::ePLANE) {
        normal=b.q.rotate(PxVec3(1,0,0));
        separation=(a.p-b.p).dot(normal)-ball.radius;
        point=a.p-normal*ball.radius;
    } else {
        float radius;
        if(other.getType()==PxGeometryType::eSPHERE) radius=static_cast<const PxSphereGeometry&>(other).radius;
        else if(other.getType()==PxGeometryType::eCUSTOM && otherObject &&
                static_cast<const PxCustomGeometry&>(other).callbacks->getCustomType()==ball.type)
            radius=static_cast<Ball*>(otherObject)->radius;
        else return false;
        normal=a.p-b.p;const float length=normal.normalize();
        if(length<1e-6f) normal=PxVec3(0,1,0);
        separation=length-ball.radius-radius;point=a.p-normal*ball.radius;
    }
    if(separation>distance) return false;
    ++contacts;return out.contact(point,normal,separation);
}
void mass(const PxGeometry&,PxMassProperties& output,void* object) {
    const float r=static_cast<Ball*>(object)->radius;
    const float m=(4.0f/3.0f)*PxPi*r*r*r;
    output.mass=m;output.centerOfMass=PxVec3(0);output.inertiaTensor=PxMat33::createDiagonal(PxVec3(.4f*m*r*r));
}
bool manifold(const PxGeometry&,float& threshold,void* object) {threshold=.02f*static_cast<Ball*>(object)->radius;return false;}
uint32_t raycast(const PxVec3& origin,const PxVec3& direction,const PxGeometry&,const PxTransform& pose,
                 float distance,PxHitFlags flags,uint32_t count,PxGeomRaycastHit* hits,uint32_t stride,
                 PxRaycastThreadContext* thread,void* object) {
    ++rays;
    const Ball & ball=*static_cast<Ball*>(object);
    return ball.context->host.raycast(origin,direction,PxSphereGeometry(ball.radius),pose,distance,flags,
                                     count,hits,stride,PxGeometryQueryFlag::eDEFAULT,thread);
}
bool overlap(const PxGeometry&,const PxTransform& pose,const PxGeometry& other,const PxTransform& otherPose,
             PxOverlapThreadContext* thread,void* object) {
    const Ball & ball=*static_cast<Ball*>(object);
    ++overlaps;
    // Delegation back into another custom geometry must be handled by that plugin.
    if(other.getType()==PxGeometryType::eCUSTOM) return false;
    return ball.context->host.overlap(PxSphereGeometry(ball.radius),pose,other,otherPose,PxGeometryQueryFlag::eDEFAULT,thread);
}
bool sweep(const PxVec3& direction,float distance,const PxGeometry&,const PxTransform& pose,
           const PxGeometry& other,const PxTransform& otherPose,PxGeomSweepHit& hit,PxHitFlags flags,
           float inflation,PxSweepThreadContext* thread,void* object) {
    const Ball & ball=*static_cast<Ball*>(object);
    ++sweeps;
    if(other.getType()==PxGeometryType::eCUSTOM) return false;
    // Custom callbacks sweep the other geometry against this stationary shape.
    return ball.context->host.sweep(direction,distance,other,otherPose,PxSphereGeometry(ball.radius),pose,
                                   hit,flags,inflation,PxGeometryQueryFlag::eDEFAULT,thread);
}
void visualize(const PxGeometry&,const PxTransform& pose,PxRenderOutput& output,const PxBounds3&,void* object) {
    ++draws;
    const float r=static_cast<Ball*>(object)->radius;
    output<<pose<<PxDebugColor::eARGB_CYAN<<PxRenderOutput::LINES;
    for(uint32_t axis=0;axis<3;++axis) {PxVec3 p(0);p[axis]=r;output<<-p<<p;}
}
const sdk::ParameterSpec radius=[] {
    sdk::ParameterSpec p{};p.name="companion:radius";p.kind=p.fallback.kind=sdk::ParameterKind::Float;
    p.fallback.scalar=.5f;return p;
}();
sdk::Plugin descriptor() {
    sdk::Plugin p;p.kind=sdk::Kind::Geometry;p.schema="CompanionBallAPI";p.parameters=&radius;p.parameterCount=1;
    p.start=[](const sdk::HostApi& host) noexcept -> void* {Context * c=new(std::nothrow) Context{host};if(c)++starts;return c;};
    p.stop=[](void* context) noexcept {++stops;delete static_cast<Context*>(context);};
    p.geometry={create,release,bounds,generate,mass,manifold,raycast,overlap,sweep,visualize};
#ifdef COMPANION_TEST_NO_QUERIES
    p.geometry.raycast=nullptr; p.geometry.overlap=nullptr; p.geometry.sweep=nullptr; p.geometry.visualize=nullptr;
#endif
#ifdef COMPANION_TEST_BAD_REQUIRED
    p.geometry.mass=nullptr;
#endif
    return p;
}
}
OVPHYSX_COMPANION_PLUGIN_EXPORT const sdk::Plugin ovphysx_companion_plugin_v1=descriptor();
OVPHYSX_COMPANION_PLUGIN_EXPORT uint32_t companion_sample_count(uint32_t which) {
    switch(which){case 0:return creates;case 1:return releases;case 2:return contacts;case 3:return starts;case 4:return stops;case 5:return rays;case 6:return overlaps;case 7:return sweeps;case 8:return draws;default:return 0;}
}

// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-GEOMETRY-001
// @covers AC-1 AC-2 AC-3
#include "ball.cpp"
#include <iostream>
#define CHECK_CONTRACT(expr) do { if (!(expr)) { std::cerr << #expr << '\n'; return 1; } } while(false)
int main() {
    sdk::ParameterValue value{};value.scalar=.75f;
    sdk::ObjectContext input{1,2,&value,1};
    const PxCustomGeometry::Type type=PxCustomGeometry::Type::INVALID();
    void * object=create(input,type,nullptr);
    CHECK_CONTRACT(object);
    struct Finally {void* object;~Finally(){release(object,nullptr);}} cleanup{object};
    PxSphereGeometry placeholder(.75f);
    const PxBounds3 box=bounds(placeholder,object);
    CHECK_CONTRACT(box.minimum==PxVec3(-.75f) && box.maximum==PxVec3(.75f));
    PxMassProperties properties;
    mass(placeholder,properties,object);
    const float expected=(4.f/3.f)*PxPi*.75f*.75f*.75f;
    CHECK_CONTRACT(PxAbs(properties.mass-expected)<1e-6f);
    CHECK_CONTRACT(properties.centerOfMass==PxVec3(0));
    CHECK_CONTRACT(PxAbs(properties.inertiaTensor.column0.x-.4f*expected*.75f*.75f)<1e-6f);
    PxContactBuffer contacts;contacts.reset();
    CHECK_CONTRACT(generate(placeholder,PxPlaneGeometry(),PxTransform(PxVec3(.7f,0,0)),PxTransform(PxIdentity),
                            .02f,0,1,contacts,object,nullptr));
    CHECK_CONTRACT(contacts.count==1 && PxAbs(contacts.contacts[0].separation+.05f)<1e-6f);
    float threshold=0;
    CHECK_CONTRACT(!manifold(placeholder,threshold,object) && threshold==.015f);
    return 0;
}

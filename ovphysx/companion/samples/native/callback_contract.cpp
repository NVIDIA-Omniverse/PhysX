// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-JOINT-001
// @covers AC-1 AC-3
// Standalone callback contract fixture: same sample implementation, no SDK owner
// or static linkage. This complements actual runtime solver/contact integration.
#include "spring.cpp"
#include <cstring>
#include <iostream>
#define CHECK_CONTRACT(expr) do { if (!(expr)) { std::cerr << #expr << '\n'; return 1; } } while(false)
int main() {
    sdk::ParameterValue parameters[3]{};
    parameters[0].scalar=100;parameters[1].scalar=2;parameters[2].scalar=1;
    const sdk::ObjectContext context{1,2,parameters,3};
    uint32_t flags=99;
    const PxTransform frame0(PxVec3(5,6,7)),frame1(PxVec3(8,6,7));
    void * object=create(context,nullptr,frame0,nullptr,frame1,flags,nullptr);
    CHECK_CONTRACT(object && flags==0);
    struct Finally {void* object;~Finally(){release(object,nullptr);}} cleanup{object};
    CHECK_CONTRACT(reinterpret_cast<uintptr_t>(block(object))%alignof(Data)==0);
    CHECK_CONTRACT(prepare(object)==block(object));
    struct Guarded {uint64_t before;Px1DConstraint rows[2];uint64_t after;} storage;
    std::memset(&storage,0xa5,sizeof(storage));
    unsigned char copy[sizeof(storage)];std::memcpy(copy,&storage,sizeof(storage));
    PxVec3p offset,ca,cb;PxConstraintInvMassScale mass{};
    const PxTransform identity(PxIdentity);
    CHECK_CONTRACT(solver(storage.rows,offset,0,mass,block(object),identity,identity,false,ca,cb)==0);
    CHECK_CONTRACT(std::memcmp(copy,&storage,sizeof(storage))==0);
    CHECK_CONTRACT(solver(storage.rows,offset,1,mass,block(object),identity,identity,false,ca,cb)==1);
    CHECK_CONTRACT(storage.before==0xa5a5a5a5a5a5a5a5ull && storage.after==storage.before);
    CHECK_CONTRACT(std::memcmp(&storage.rows[1],copy+offsetof(Guarded,rows)+sizeof(Px1DConstraint),sizeof(Px1DConstraint))==0);
    CHECK_CONTRACT(ca==frame0.p && cb==frame1.p);
    CHECK_CONTRACT(storage.rows[0].geometricError==2 && mass.linear0==1 && mass.angular1==1);
    originShift(object,PxVec3(1,2,3));
    comShift(object,0);comShift(object,1);
    CHECK_CONTRACT(solver(storage.rows,offset,1,mass,block(object),identity,identity,false,ca,cb)==1);
    CHECK_CONTRACT(ca==PxVec3(4,4,4) && cb==PxVec3(7,4,4));
    CHECK_CONTRACT(storage.rows[0].geometricError==2);
    return 0;
}

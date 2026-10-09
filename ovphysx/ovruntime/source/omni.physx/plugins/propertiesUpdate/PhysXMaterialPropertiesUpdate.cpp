// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PROPS-MAT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

#include "PhysXPropertiesUpdate.h"
#include "usdLoad/Material.h"
#include "PhysXScene.h"

#include <omni/physics/parse/KnownTokens.h>

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

// A relationship edit changes the shape's material, not the material's coefficients.
// Use the source resolver so physics-purpose precedence and default fallback agree
// with initial parsing. Only collider records are handled by this callback.
bool omni::physx::updateShapeMaterialBinding(AttachedStage& attachedStage, ObjectId objectId,
    omni::physics::parse::TokenId, omni::physics::parse::ReadTime)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType type = ePTRemoved;
    const InternalDatabase::Record* record = db.getFullRecord(type, objectId);
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!record || !source || (type != ePTShape && type != ePTCompoundShape))
        return true;

    InternalShape* internalShape = static_cast<InternalShape*>(record->mInternalPtr);
    std::vector<PxShape*> shapes;
    PhysXScene* scene = nullptr;
    ObjectId* trackedMaterial = nullptr;
    if (type == ePTShape)
    {
        shapes.push_back(static_cast<PxShape*>(record->mPtr));
        scene = internalShape->mPhysicsScene;
        trackedMaterial = &internalShape->mMaterialId;
    }
    else
    {
        CompoundShape* compound = static_cast<CompoundShape*>(record->mPtr);
        shapes = compound->getShapes();
        scene = compound->mPhysXScene;
        trackedMaterial = &compound->mMaterialId;
    }

    // PhysX rejects writes to attached shared shapes. Reject the whole collider
    // before changing any slot or reverse ownership record.
    for (const PxShape* shape : shapes)
    {
        if (!shape->isExclusive())
        {
            CARB_LOG_WARN("Live material rebinding is not supported for shared shapes: %s",
                attachedStage.textFor(record->mKey));
            return true;
        }
    }

    const omni::physics::parse::ObjectKey materialKey =
        internalShape->mSourceGprim.valid() ? internalShape->mSourceGprim : record->mKey;
    ObjectId materialId = getMaterial(attachedStage, source->getMaterialBinding(materialKey));
    const InternalDatabase::Record* materialRecord = db.getFullTypedRecord(ePTMaterial, materialId);
    PxMaterial* material = materialRecord ? static_cast<PxMaterial*>(materialRecord->mPtr) : scene->getDefaultMaterial();
    if (!materialRecord)
        materialId = kInvalidObjectId;

    // Tensor writes can replace the live material while retaining its authored binding.
    // Invalid IDs cannot establish equality: deleting a material also clears its ID.
    const bool bindingUnchanged = materialId != kInvalidObjectId && *trackedMaterial == materialId;
    for (PxShape* shape : shapes)
    {
        // Mesh subset slots are unchanged by a collider-level binding edit; the
        // collider's own material occupies the trailing fallback slot.
        std::vector<PxMaterial*> materials(shape->getNbMaterials());
        shape->getMaterials(materials.data(), PxU32(materials.size()));
        // An all-purpose edit can leave the effective physics binding unchanged.
        // In that case preserve runtime mass/inertia and avoid an unnecessary write.
        if (bindingUnchanged || materials.back() == material)
            continue;
        materials.back() = material;
        shape->setMaterials(materials.data(), PxU16(materials.size()));
        if (PxRigidActor* actor = shape->getActor())
            db.addDirtyMassActor(size_t(actor->userData));
    }

    // Coefficient/density updates and destruction use this reverse ownership list.
    // Rebinding to the same material must not accumulate duplicate shape entries.
    if (*trackedMaterial != materialId)
    {
        const InternalDatabase::Record* previous = db.getFullTypedRecord(ePTMaterial, *trackedMaterial);
        if (previous && previous->mInternalPtr)
            static_cast<InternalMaterial*>(previous->mInternalPtr)->removeShapeId(objectId);
        if (materialRecord && materialRecord->mInternalPtr)
            static_cast<InternalMaterial*>(materialRecord->mInternalPtr)->addShapeId(objectId);
        *trackedMaterial = materialId;
    }
    // Compound colliders also carry an InternalShape used by replication.
    static_cast<InternalShape*>(record->mInternalPtr)->mMaterialId = materialId;
    return true;
}

// physx material
bool omni::physx::updateMaterialFrictionCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        if (!source)
            return true;

        omni::physics::parse::TokenId data;
        if (!source->getAttribute(objectRecord->mKey, property, data))
            return true;

        omni::physics::parse::KnownTokens tok;
        tok.intern(*source);

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (tok.average == data)
            material->setFrictionCombineMode(PxCombineMode::eAVERAGE);
        else if (tok.min == data)
            material->setFrictionCombineMode(PxCombineMode::eMIN);
        else if (tok.max == data)
            material->setFrictionCombineMode(PxCombineMode::eMAX);
        else if (tok.multiply == data)
            material->setFrictionCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateMaterialRestitutionCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        if (!source)
            return true;

        omni::physics::parse::TokenId data;
        if (!source->getAttribute(objectRecord->mKey, property, data))
            return true;

        omni::physics::parse::KnownTokens tok;
        tok.intern(*source);

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (tok.average == data)
            material->setRestitutionCombineMode(PxCombineMode::eAVERAGE);
        else if (tok.min == data)
            material->setRestitutionCombineMode(PxCombineMode::eMIN);
        else if (tok.max == data)
            material->setRestitutionCombineMode(PxCombineMode::eMAX);
        else if (tok.multiply == data)
            material->setRestitutionCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateMaterialDampingCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        if (!source)
            return true;

        omni::physics::parse::TokenId data;
        if (!source->getAttribute(objectRecord->mKey, property, data))
            return true;

        omni::physics::parse::KnownTokens tok;
        tok.intern(*source);

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (tok.average == data)
            material->setDampingCombineMode(PxCombineMode::eAVERAGE);
        else if (tok.min == data)
            material->setDampingCombineMode(PxCombineMode::eMIN);
        else if (tok.max == data)
            material->setDampingCombineMode(PxCombineMode::eMAX);
        else if (tok.multiply == data)
            material->setDampingCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateCompliantMaterial(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
                                          omni::physics::parse::TokenId property,
                                          omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        PxMaterial* material = reinterpret_cast<PxMaterial*>(objectRecord->mPtr);
        if (material)
        {
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.compliantContactStiffness)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                if (data > 0.0f)
                {
                    material->setRestitution(-data);  // negative restitution is interpreted as compliant stiffness
                }
                else
                {
                    // disable compliance and restore restitution from USD:
                    float restitution = 0.0f;
                    getValue<float>(attachedStage, objectRecord->mKey, tok.restitution, timeCode, restitution);
                    material->setRestitution(restitution);
                }
            }
            else if (property == tok.compliantContactDamping)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                if(material->getRestitution() >= 0.0f) // restitution < 0 implies compliant contact behavior
                {
                    CARB_LOG_WARN(
                        "Updating compliant contact damping on material %s, but compliant stiffness is zero. Set stiffness >0 first to enable compliance.",
                        attachedStage.textFor(objectRecord->mKey));
                    return true;
                }
                material->setDamping(data);
            }
            else if (property == tok.compliantContactAccelerationSpring)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                if(material->getRestitution() >= 0.0f) // restitution < 0 implies compliant contact behavior
                {
                    CARB_LOG_WARN(
                        "Updating compliant contact acceleration spring on material %s, but compliant stiffness is zero. Set stiffness >0 first to enable compliance.",
                        attachedStage.textFor(objectRecord->mKey));
                    return true;
                }
                material->setFlag(PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING, data);
            }
        }
    }
    return true;
}

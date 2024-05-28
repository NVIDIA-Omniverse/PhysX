// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef NP_ARTICULATION_TENDON_H
#define NP_ARTICULATION_TENDON_H

#include "foundation/PxInlineArray.h"
#include "PxArticulationTendon.h"
#include "ScArticulationTendonCore.h"
#include "ScArticulationAttachmentCore.h"
#include "ScArticulationTendonJointCore.h"
#include "NpBase.h"


namespace physx
{

typedef PxU32	ArticulationAttachmentHandle;
typedef PxU32	ArticulationTendonHandle;

class NpArticulationReducedCoordinate;

class NpArticulationAttachment;
class NpArticulationSpatialTendon;

class NpArticulationTendonJoint;
class NpArticulationFixedTendon;

class NpArticulationAttachmentArray : public PxInlineArray<NpArticulationAttachment*, 4>  //!!!AL TODO: check if default of 4 elements makes sense
{
public:
	// PX_SERIALIZATION
	NpArticulationAttachmentArray(const PxEMPTY) : PxInlineArray<NpArticulationAttachment*, 4>(PxEmpty) {}
	static	void	getBinaryMetaData(PxOutputStream& stream);
	//~PX_SERIALIZATION
	NpArticulationAttachmentArray() : PxInlineArray<NpArticulationAttachment*, 4>("NpArticulationAttachmentArray") {}
};

class NpArticulationTendonJointArray : public PxInlineArray<NpArticulationTendonJoint*, 4>  //!!!AL TODO: check if default of 4 elements makes sense
{
public:
	// PX_SERIALIZATION
	NpArticulationTendonJointArray(const PxEMPTY) : PxInlineArray<NpArticulationTendonJoint*, 4>(PxEmpty) {}
	static	void	getBinaryMetaData(PxOutputStream& stream);
	//~PX_SERIALIZATION
	NpArticulationTendonJointArray() : PxInlineArray<NpArticulationTendonJoint*, 4>("NpArticulationTendonJointArray") {}
};

class NpArticulationAttachment : public PxArticulationAttachment, public NpBase
{

public:

// PX_SERIALIZATION
											NpArticulationAttachment(PxBaseFlags baseFlags)
													: PxArticulationAttachment(baseFlags), NpBase(PxEmpty), mHandle(PxEmpty), mChildren(PxEmpty), mCore(PxEmpty) {}
				void						preExportDataReset() { mCore.preExportDataReset(); }
	virtual		void						exportExtraData(PxSerializationContext& );
				void						importExtraData(PxDeserializationContext& );
				void						resolveReferences(PxDeserializationContext& );
	virtual		void						requiresObjects(PxProcessPxBaseCallback&);
	virtual		bool						isSubordinate()  const	 { return true; } 
	static		NpArticulationAttachment*	createObject(PxU8*& address, PxDeserializationContext& context);
	static		void						getBinaryMetaData(PxOutputStream& stream);		
//~PX_SERIALIZATION


	NpArticulationAttachment(PxArticulationAttachment* parent, const PxReal coefficient, const PxVec3 relativeOffset, PxArticulationLink* link);

	virtual ~NpArticulationAttachment();


	virtual		void							setRestLength(const PxReal restLength);
	virtual		PxReal							getRestLength() const;

	virtual		void							setLimitParameters(const PxArticulationTendonLimit& paramters);
	virtual		PxArticulationTendonLimit		getLimitParameters() const;

	virtual		void							setRelativeOffset(const PxVec3& offset);
	virtual		PxVec3							getRelativeOffset() const;

	virtual		void							setCoefficient(const PxReal coefficient);
	virtual		PxReal							getCoefficient() const;

	virtual		PxArticulationLink*				getLink() const { return mLink; }
	virtual		PxArticulationAttachment*		getParent() const { return mParent; }
	virtual		PxArticulationSpatialTendon*	getTendon() const;
	
	virtual		bool							isLeaf() const { return mChildren.empty(); }

	virtual		void							release();

	PX_FORCE_INLINE NpArticulationAttachment** getChildren() { return mChildren.begin(); }
	PX_FORCE_INLINE PxU32 getNumChildren() { return mChildren.size(); }

	PX_FORCE_INLINE void setTendon(NpArticulationSpatialTendon* tendon) { mTendon = tendon; }
	PX_FORCE_INLINE NpArticulationSpatialTendon& getTendon() { return *mTendon; }

	PX_FORCE_INLINE Sc::ArticulationAttachmentCore&	getCore() { return mCore;  }

	void removeChild(NpArticulationAttachment* child);

	PxArticulationLink*					mLink;				//the link this attachment attach to 
	PxArticulationAttachment*			mParent;
	ArticulationAttachmentHandle		mHandle;
	NpArticulationAttachmentArray		mChildren;
	NpArticulationSpatialTendon*		mTendon;
	Sc::ArticulationAttachmentCore		mCore;
};


class NpArticulationSpatialTendon : public PxArticulationSpatialTendon, public NpBase
{
public:

// PX_SERIALIZATION
											NpArticulationSpatialTendon(PxBaseFlags baseFlags)
													: PxArticulationSpatialTendon(baseFlags), NpBase(PxEmpty), mAttachments(PxEmpty), mCore(PxEmpty) {}
				void						preExportDataReset() { mCore.preExportDataReset(); }
	virtual		void						exportExtraData(PxSerializationContext& );
				void						importExtraData(PxDeserializationContext& );
				void						resolveReferences(PxDeserializationContext& );
	virtual		void						requiresObjects(PxProcessPxBaseCallback&);
	virtual		bool						isSubordinate()  const	 { return true; } 
	static		NpArticulationSpatialTendon* createObject(PxU8*& address, PxDeserializationContext& context);
	static		void						getBinaryMetaData(PxOutputStream& stream);		
//~PX_SERIALIZATION

	NpArticulationSpatialTendon(NpArticulationReducedCoordinate* articulation);

	virtual ~NpArticulationSpatialTendon();

	virtual		PxArticulationAttachment*		createAttachment(PxArticulationAttachment* parent, const PxReal coefficient, const PxVec3 relativeOffset, PxArticulationLink* link);

	NpArticulationAttachment*					getAttachment(const PxU32 index);
	virtual		PxU32							getAttachments(PxArticulationAttachment** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;
	PxU32										getNbAttachments() const;

	virtual		void							setStiffness(const PxReal stiffness);
	virtual		PxReal							getStiffness() const;

	virtual		void							setDamping(const PxReal damping);
	virtual		PxReal							getDamping() const;

	virtual		void							setLimitStiffness(const PxReal stiffness);
	virtual		PxReal							getLimitStiffness() const;

	virtual		void							setOffset(const PxReal offset, bool autowake = true);
	virtual		PxReal							getOffset() const;

	virtual		PxArticulationReducedCoordinate* getArticulation() const;

	virtual		void							release();

	PX_FORCE_INLINE Sc::ArticulationSpatialTendonCore&	getTendonCore() { return mCore; }

	PX_FORCE_INLINE ArticulationTendonHandle			getHandle() { return mHandle; }
	PX_FORCE_INLINE void								setHandle(ArticulationTendonHandle handle) { mHandle = handle; }

	PX_FORCE_INLINE NpArticulationAttachmentArray&		getAttachments() { return mAttachments; }

private:
	NpArticulationAttachmentArray				mAttachments;
	NpArticulationReducedCoordinate*			mArticulation;
	PxU32										mLLIndex;
	Sc::ArticulationSpatialTendonCore			mCore;
	ArticulationTendonHandle					mHandle;
};

class NpArticulationTendonJoint  : public PxArticulationTendonJoint, public NpBase
{
public:
	// PX_SERIALIZATION
	NpArticulationTendonJoint(PxBaseFlags baseFlags) : PxArticulationTendonJoint(baseFlags), NpBase(PxEmpty), mChildren(PxEmpty), mCore(PxEmpty), mHandle(PxEmpty) {}
	void						preExportDataReset() { mCore.preExportDataReset(); }
	virtual		void			exportExtraData(PxSerializationContext& );
	void						importExtraData(PxDeserializationContext& );
	void						resolveReferences(PxDeserializationContext& );
	virtual		void			requiresObjects(PxProcessPxBaseCallback&);
	virtual		bool			isSubordinate()  const	 { return true; } 
	static		NpArticulationTendonJoint*	createObject(PxU8*& address, PxDeserializationContext& context);
	static		void						getBinaryMetaData(PxOutputStream& stream);		
	//~PX_SERIALIZATION

	NpArticulationTendonJoint(PxArticulationTendonJoint* parent, PxArticulationAxis::Enum axis, const PxReal coefficient, const PxReal recipCoefficient, PxArticulationLink* link);

	virtual ~NpArticulationTendonJoint(){}

	virtual		PxArticulationLink*					getLink() const { return mLink; }

	virtual		PxArticulationTendonJoint*			getParent() const { return mParent;  }

	virtual		PxArticulationFixedTendon*			getTendon() const;

	virtual		void								setCoefficient(const PxArticulationAxis::Enum axis, const PxReal coefficient, const PxReal recipCoefficient);
	virtual		void								getCoefficient(PxArticulationAxis::Enum& axis, PxReal& coefficient, PxReal& recipCoefficient) const;


	virtual		void								release();

	PX_FORCE_INLINE NpArticulationTendonJoint** getChildren() { return mChildren.begin(); }
	PX_FORCE_INLINE PxU32 getNumChildren() { return mChildren.size(); }

	PX_FORCE_INLINE void setTendon(NpArticulationFixedTendon* tendon) { mTendon = tendon; }
	PX_FORCE_INLINE NpArticulationFixedTendon& getTendon() { return *mTendon; }

	PX_FORCE_INLINE Sc::ArticulationTendonJointCore&	getCore() { return mCore; }

	void removeChild(NpArticulationTendonJoint* child);

	PxArticulationLink*					mLink;				//the link this joint associated with 
	PxArticulationTendonJoint*			mParent;
	NpArticulationTendonJointArray		mChildren;
	NpArticulationFixedTendon*			mTendon;
	Sc::ArticulationTendonJointCore		mCore;
	PxU32								mHandle;
	
};

class NpArticulationFixedTendon : public PxArticulationFixedTendon, public NpBase
{
public:
	// PX_SERIALIZATION
	NpArticulationFixedTendon(PxBaseFlags baseFlags): PxArticulationFixedTendon(baseFlags), NpBase(PxEmpty), mTendonJoints(PxEmpty), mCore(PxEmpty), mHandle(PxEmpty) {}
	void						preExportDataReset() {}
	virtual		void			exportExtraData(PxSerializationContext& );
	void						importExtraData(PxDeserializationContext& );
	void						resolveReferences(PxDeserializationContext& );
	virtual		void						requiresObjects(PxProcessPxBaseCallback&);
	virtual		bool						isSubordinate()  const	 { return true; } 
	static		NpArticulationFixedTendon* createObject(PxU8*& address, PxDeserializationContext& context);
	static		void						getBinaryMetaData(PxOutputStream& stream);
	//~PX_SERIALIZATION

	NpArticulationFixedTendon(NpArticulationReducedCoordinate* articulation);

	virtual ~NpArticulationFixedTendon();

	virtual		PxArticulationTendonJoint*		createTendonJoint(PxArticulationTendonJoint* parent, PxArticulationAxis::Enum axis, const PxReal scale, const PxReal recipScale, PxArticulationLink* link);

	NpArticulationTendonJoint*					getTendonJoint(const PxU32 index);
	virtual		PxU32							getTendonJoints(PxArticulationTendonJoint** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;
	virtual		PxU32							getNbTendonJoints(void) const;

	virtual		void							setStiffness(const PxReal stiffness);
	virtual		PxReal							getStiffness() const;

	virtual		void							setDamping(const PxReal damping);
	virtual		PxReal							getDamping() const;

	virtual		void							setLimitStiffness(const PxReal damping);
	virtual		PxReal							getLimitStiffness() const;

	virtual		void							setRestLength(const PxReal restLength);
	virtual		PxReal							getRestLength() const;

	virtual		void							setLimitParameters(const PxArticulationTendonLimit& paramters);
	virtual		PxArticulationTendonLimit		getLimitParameters() const;

	virtual		void							setOffset(const PxReal offset, bool autowake = true);
	virtual		PxReal							getOffset() const;

	virtual		PxArticulationReducedCoordinate* getArticulation() const;

	virtual		void							release();

	PX_FORCE_INLINE ArticulationTendonHandle			getHandle() { return mHandle; }
	PX_FORCE_INLINE void								setHandle(ArticulationTendonHandle handle) { mHandle = handle; }

	PX_FORCE_INLINE Sc::ArticulationFixedTendonCore&			getTendonCore() { return mCore; }

	PX_FORCE_INLINE NpArticulationTendonJointArray&		getTendonJoints() { return mTendonJoints; }

private:
	NpArticulationTendonJointArray				mTendonJoints;
	NpArticulationReducedCoordinate*			mArticulation;
	PxU32										mLLIndex;
	Sc::ArticulationFixedTendonCore				mCore;
	ArticulationTendonHandle					mHandle;
};

}

#endif

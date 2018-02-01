
// (c) Micelanholies 2015
// Micelanholies Physics Engine
// BVH3 - Bounding Volume Hierarchy 3d

#ifndef	__MPE_BVH3__
#define	__MPE_BVH3__

#include "MpeVec3.h"
#include "MpeAABB3.h"


namespace Mpe
{

	template <class callBack, typename type, typename data> class BVH3
	{

	public:
		static const _ui  s_numChildren	= 2;							// total number of children per node
		static const type s_avgCoordTolerance = (const type)0.0001;		// coordinate compare tolerance for averages

		struct Elem
		{
			_ui			elemId;											// index of element of leaf
			_ui			elemType;										// type of element of leaf
			Vec3<type>	avg;											// average of leaf (central point) or average of all children of node
			AABB3<type>	aabb;											// AABB of leaf or AABB of all children nodes of node
			AABB3<type>	aabbAvg;										// AABB of subtraction or AABB of all children averages 
			data*		pData;											// pointer to callBack holder data
		};

	private:
		static const _ui s_childLId		= 0;							// index of left child
		static const _ui s_childRId		= 1;							// index of right child

		struct Node
		{
			_ui			parentId;										// index of parent node
			_ui			childId[s_numChildren];							// index of child nodes
			_ui			level;											// distance from root node
			_ui			maxLeafDistance;								// maximal distance to leaf (0 for leaf)
			_ui			parentChildId;									// index of this node of parent as a child
			_ui			numAvg;											// number of all children averages
			_ui			uid;											// unique index of node (or base element)
			Elem		elem;											// element data of node
		};


	private:
		Node*	_pNode;													// nodes of BVH
		_ui*	_pFreeNode;												// indices of free nodes
		_ui		_numNodesMax;											// maximal number of nodes
		_ui		_numNodes;												// number of defined nodes
		_ui		_numFreeNodes;											// number of free nodes inside of nodes array
		_ui		_rootNodeId;											// index of parent node of all nodes

	public:
		BVH3(void);
		BVH3(_ui numNodesMax);
		~BVH3(void);

		typedef _b (callBack::*callBackIntersectionFunc)(const Elem& elem, const Elem& elemBVH);
		typedef _b (callBack::*callBackUpdateFunc)(Elem& elem);


		_b   __fastcall		init(_ui numNodesMax);

		_ui  __fastcall		numNodesMax(void) const;
		_ui  __fastcall		numNodes(void) const;
		_ui  __fastcall		numFreeNodes(void) const;

		_b   __fastcall		define(const Elem* pElem, _ui numElements);

		_ui  __fastcall		push(const Elem& elem);
		_b   __fastcall		build(void);

		_ui  __fastcall		add(const Elem& elem);						// add new element to BVH, return index of element node
		_b   __fastcall		del(_ui nodeId);							// delete node of element, false if not a leaf
		_b   __fastcall		get(_ui nodeId, Elem& elem) const;			// get element of node, false if not a leaf
		_ui  __fastcall		set(_ui nodeId, Elem& elem);				// set element of node, update BVH, false if not a leaf
		_b   __fastcall		check(const Elem& elem, _b bOneIntersection, _b bSubAvg, callBack& callBackClass, callBackIntersectionFunc intersectionFunc) const;
		_b   __fastcall		exist(_ui nodeId) const;

		_b   __fastcall		update(callBack& callBackClass, callBackUpdateFunc updateFunc);

		_b   __fastcall		verify(void) const;

	private:
		void __fastcall		reset(void);
		void __fastcall		flush(void);

		_ui  __fastcall		addNodeRaw(void);																				// O(1)
		void __fastcall		delNodeRaw(_ui nodeId);																			// O(1)

		_ui  __fastcall		getNeighborNode(_ui nodeId) const;																// O(1)
		_ui  __fastcall		getDownNode(_ui nodeId, _ui path, _ui depth) const;												// O(log N)

		_ui  __fastcall		axisIndex(_ui level) const;																		// O(1)
		type __fastcall		getAxis(const Vec3<type>& vec, _ui axisId) const;												// O(1)
		_ui  __fastcall		switchAxis(const Vec3<type>& avgLeaf, const Vec3<type>& avgBranch, _ui level) const;			// O(1)
		
		_ui  __fastcall		searchLeaf(_ui nodeId, const Elem& elem) const;													// O(log N)
		_ui  __fastcall		searchOutLeaf(_ui nodeId, _ui axis, const type& avg, const type& sign) const;					// O(log N)

		_b   __fastcall		avgIntersection(_ui nodeParentId, _ui childId, _ui nodeId) const;								// O(1)
		_b   __fastcall		avgIntersection(_ui nodeId, _ui childId) const;													// O(1)

		_ui  __fastcall		addLeaf(const Elem& elem, _ui nodeId, _ui childId);												// O(1)
		_b   __fastcall		delLeaf(_ui nodeId);																			// O(1)
		_b   __fastcall		moveLeaf(_ui nodeFromId, _ui nodeToId);															// O(1)

		void __fastcall		updateElem(_ui nodeId);																			// O(1)
		void __fastcall		updateNode(_ui nodeId);																			// O(1)

		void __fastcall		updateRoot(_ui nodeId);																			// O(1)
		void __fastcall		updateParent(_ui nodeId);																		// O(1)
		void __fastcall		updateChildren(_ui nodeId);																		// O(N log N)
		_b   __fastcall		recalculate(_ui nodeId);																		// O(N)
		_b   __fastcall		restructurize(_ui nodeId, _ui rootNodeId, _ui childSwap);										// O(N log N)

		void __fastcall		sortX(_ui iLo, _ui iHi);																		// O(N log N)
		void __fastcall		sortY(_ui iLo, _ui iHi);																		// O(N log N)
		void __fastcall		sortZ(_ui iLo, _ui iHi);																		// O(N log N)

		_ui  __fastcall		sort(_ui iLo, _ui iHi, _ui level);																// O(N log2 N)

		void __fastcall		swapLeaf(_ui iFrom, _ui iTo);																	// O(1)
		void __fastcall		sortByUid(_ui iLO, _ui iHi);																	// O(N log N)

		void __fastcall		update(_ui nodeId);																				// O(N log N)

		_b   __fastcall		intersection(const Node& node, _ui nodeId) const;												// O(1)

		_b   __fastcall		check(const Elem& elem, _ui nodeId, _b& bIntersection, _b bOneIntersection, _b bSubAvg,			// O(N log N)
									callBack& callBackClass, callBackIntersectionFunc intersectionFunc) const;

		_b   __fastcall		update(_ui nodeId, callBack& callBackClass, callBackUpdateFunc updateFunc);						// O(N log N)
	};




	//template <class callBack, typename type, typename data> const type BVH3<callBack,type,data>::s_avgCoordTolerance = (const type)0.0001;			// coordinate compare tolerance for averages





	template <class callBack, typename type, typename data> BVH3<callBack, type, data>::BVH3(void)
	{
		reset();
	}

	template <class callBack, typename type, typename data> BVH3<callBack, type, data>::BVH3(_ui numNodesMax)
	{
		init(numNodesMax);
	}

	template <class callBack, typename type, typename data> BVH3<callBack, type, data>::~BVH3(void)
	{
		flush();
	}

	

	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::init(_ui numNodesMax)
	{
		flush();
		try
		{
			_pNode		= new Node[numNodesMax];
			_pFreeNode	= new _ui[numNodesMax];
		}
		catch(...)
		{
			flush();
			return false;
		}
		for (_ui id = 0; id<numNodesMax; id++)
			_pNode[id].uid = id;
		_numNodesMax	= numNodesMax;
		_numFreeNodes	= 0;
		return true;
	}


	
	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::numNodesMax(void) const
	{
		return _numNodesMax;
	}

	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::numNodes(void) const
	{
		return _numNodes;
	}

	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::numFreeNodes(void) const
	{
		return _numFreeNodes;
	}



	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::define(const Elem* pElem, _ui numElements)
	{
		if (numElements>=_numNodesMax) return false;
		for (_ui id = 0; id<numElements; id++)
			if (push(pElem[id])>=_numNodesMax) return false;
		_numNodes = numElements;
		return build();
	}



	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::push(const Elem& elem)
	{
		if (_numNodes>=_numNodesMax) return _numNodesMax;
		const _ui nodeId = _numNodes;
		Node& node = _pNode[nodeId];
		node.elem					= elem;
		node.parentId				= _numNodesMax;
		node.childId[s_childLId]	= _numNodesMax;
		node.childId[s_childRId]	= _numNodesMax;
		node.level					= 0;
		node.maxLeafDistance		= 0;
		node.parentChildId			= _numNodesMax;
		node.numAvg					= 1;
		node.elem.aabbAvg			= node.elem.avg;
		_numNodes++;
		return nodeId;
	}
	
	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::build(void)
	{
		_ui uid = 0;
		for (_ui nodeId = 0; nodeId<_numNodes; nodeId++)
			_pNode[nodeId].uid = _pNode[nodeId].maxLeafDistance==0 ? uid++ : _numNodesMax;
		const _ui numNodes = uid;
		const _ui numNodesMax = _numNodes * (_ui)(Math::log<_d>((_d)_numNodes,(_d)2));
		if (numNodesMax>_numNodesMax) return false;
		_rootNodeId = sort(0, numNodes-1, 0);
		if (_rootNodeId>=_numNodesMax) return false;
		sortByUid(0, numNodes-1);
		return true;
	}
	
	
	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::add(const Elem& elem)
	{
		// set root node
		if (!_numNodes)
		{
			Node& node = _pNode[0];
			node.elem					= elem;
			node.parentId				= _numNodesMax;
			node.childId[s_childLId]	= _numNodesMax;
			node.childId[s_childRId]	= _numNodesMax;
			node.level					= 0;
			node.maxLeafDistance		= 0;
			node.parentChildId			= _numNodesMax;
			node.numAvg					= 1;
			node.elem.aabbAvg			= node.elem.avg;
			_numNodes++;
			_rootNodeId = 0;
			return _rootNodeId;
		}
		// set branch or leaf node
		const _ui nodeSId = searchLeaf(_rootNodeId, elem);
		if (nodeSId>=_numNodes) return _numNodesMax;
		Node& nodeS = _pNode[nodeSId];
		const _ui sideId = switchAxis(elem.avg, nodeS.elem.avg, nodeS.level);
		const _ui nodeNId = addLeaf(elem, nodeSId, sideId);
		if (nodeNId>=_numNodes) return _numNodesMax;
		return nodeNId;
	}

	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::del(_ui nodeId)
	{
		if (nodeId>=_numNodes) return false;
		const Node& node = _pNode[nodeId];
		if (node.maxLeafDistance>0) return false;
		return delLeaf(nodeId);
	}

	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::get(_ui nodeId, Elem& elem) const
	{
		if (nodeId>=_numNodes) return false;
		const Node& node = _pNode[nodeId];
		if (node.maxLeafDistance>0) return false;
		Elem = node.elem;
		return true;
	}

	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::set(_ui nodeId, Elem& elem)
	{
		if (nodeId>=_numNodes) return false;
		const Node& node = _pNode[nodeId];
		if (node.maxLeafDistance>0) return false;
		const _ui nodeToId = searchLeaf(_rootNodeId, elem);
		return moveLeaf(nodeId, nodeToId);
	}
	
	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::check(const Elem& elem, _b bOneIntersection, _b bSubAvg, callBack& callBackClass, callBackIntersectionFunc intersectionFunc) const
	{
		_b bIntersection = false;
		return check(elem, _rootNodeId, bIntersection, bOneIntersection, bSubAvg, callBackClass, intersectionFunc);
	}

	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::exist(_ui nodeId) const
	{
		return (nodeId<_numNodes && _pNode[nodeId].level!=_numNodesMax);
	}


	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::update(callBack& callBackClass, callBackUpdateFunc updateFunc)
	{
		if (!update(_rootNodeId, callBackClass, updateFunc)) return false;
		update(_rootNodeId);
		return true;
	}


	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::verify(void) const
	{
		// check connections
		for (_ui nodeTId = 0; nodeTId<_numNodes; nodeTId++)
		{
			if (!exist(nodeTId)) continue;
			const Node& nodeT = _pNode[nodeTId];
			const _ui nodePId = nodeT.parentId;
			const _ui nodeLId = nodeT.childId[s_childLId];
			const _ui nodeRId = nodeT.childId[s_childRId];
			if (exist(nodePId) && _pNode[nodePId].childId[nodeT.parentChildId]!=nodeTId)
			{
				return false;
			}
			if (exist(nodeLId) && _pNode[nodeLId].parentId!=nodeTId)
			{
				return false;
			}
			if (exist(nodeRId) && _pNode[nodeRId].parentId!=nodeTId)
			{
				return false;
			}
		}
		// check levels
		for (_ui nodeTId = 0; nodeTId<_numNodes; nodeTId++)
		{
			if (!exist(nodeTId)) continue;
			const Node& nodeT = _pNode[nodeTId];
			const _ui nodePId = nodeT.parentId;
			if (nodePId==_numNodesMax)
			{
				if (nodeT.level!=0)
				{
					return false;
				}
				continue;
			}
			const Node& nodeP = _pNode[nodePId];
			if (nodeT.level!=nodeP.level+1)
			{
				return false;
			}
		}
		// check bounds
		for (_ui nodeTId = 0; nodeTId<_numNodes; nodeTId++)
		{
			if (!exist(nodeTId)) continue;
			if (_pNode[nodeTId].maxLeafDistance!=0) continue;
			_ui nodeCId = nodeTId;
			_ui nodePId = _pNode[nodeCId].parentId;
			while (nodePId<_numNodes)
			{
				if (!_pNode[nodePId].elem.aabb.cover(_pNode[nodeCId].elem.aabb))
				{
					return false;
				}
				nodeCId = nodePId;
				nodePId = _pNode[nodeCId].parentId;
			}
		}
		return true;
	}




	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::reset(void)
	{
		_pNode			= NULL;
		_pFreeNode		= NULL;
		_numNodes		= 0;
		_numFreeNodes	= 0;
		_numNodesMax	= 0;
		_rootNodeId		= 0;
	}

	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::flush(void)
	{
		try	{	delete[] _pNode;		}	catch(...)	{};
		try	{	delete[] _pFreeNode;	}	catch(...)	{};
		reset();
	}



	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::addNodeRaw(void)
	{
		return _numFreeNodes ? _pFreeNode[--_numFreeNodes] : _numNodes++;
	}
	
	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::delNodeRaw(_ui nodeId)
	{
		if (nodeId<_numNodes-1)	_pFreeNode[_numFreeNodes++] = nodeId;
		else					_numNodes--;
	}



	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::getNeighborNode(_ui nodeId) const
	{
		const _ui nodePId = _pNode[nodeId].parentId;
		const _ui parentNeighborChildId = (_pNode[nodeId].parentChildId + 1) % s_numChildren;
		return _pNode[nodePId].childId[parentNeighborChildId];
	}

	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::getDownNode(_ui nodeId, _ui path, _ui depth) const
	{
		_ui nodeTId = nodeId;
		for (_ui d = 0; d<depth, nodeTId<_numNodes; d++)
		{
			const _ui childId = (path & (1<<d));
			nodeTId = _pNode[nodeTId].childId[childId];
		}
		return nodeTId;
	}



	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::axisIndex(_ui level) const
	{
		return (level % 3);
	}

	template <class callBack, typename type, typename data> type __fastcall BVH3<callBack, type, data>::getAxis(const Vec3<type>& vec, _ui axisId) const
	{
		const type v[3] = { vec.x, vec.y, vec.z };
		return v[axisId];
	}

	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::switchAxis(const Vec3<type>& avgLeaf, const Vec3<type>& avgBranch, _ui level) const
	{
		const _ui axis = axisIndex(level);
		const type avgL = getAxis(avgLeaf, axis);
		const type avgB = getAxis(avgBranch, axis);
		return avgL <= avgB ? 0 : 1;
	}



	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::searchLeaf(_ui nodeId, const Elem& elem) const
	{
		_ui nodeTId = nodeId;
		for (_ui i = 0; i<_numNodes; i++)												// inf limited cycle
		{
			Node& nodeT = _pNode[nodeTId];
			const _ui sideId = switchAxis(elem.avg, nodeT.elem.avg, nodeT.level);		// get child index side of node by average point of this node
			if (nodeT.maxLeafDistance!=0)	nodeTId = nodeT.childId[sideId];			// branch node
			else							return nodeTId;								// leaf node
		}
		return _numNodesMax;
	}

	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::searchOutLeaf(_ui nodeId, _ui axis, const type& avg, const type& sign) const
	{
		if (nodeId>=_numNodes) return _numNodesMax;
		const Node& node = _pNode[nodeId];
		const type l  = getAxis(node.elem.aabbAvg.l, axis);
		const type h  = getAxis(node.elem.aabbAvg.h, axis);
		const type la = (l-avg)*sign;
		const type ha = (h-avg)*sign;
		if (la<(const type)0 || ha<(const type)0)
		{
			if (node.maxLeafDistance==0) return nodeId;
			const _ui nodeLId = searchOutLeaf(node.childId[s_childLId], axis, avg, sign);	if (nodeLId<_numNodesMax) return nodeLId;
			const _ui nodeRId = searchOutLeaf(node.childId[s_childRId], axis, avg, sign);	if (nodeRId<_numNodesMax) return nodeRId;
		}
		return _numNodesMax;
	}



	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::avgIntersection(_ui nodeId, _ui childId) const
	{
		const _ui nodeTId = nodeId;
		const Node& nodeT = _pNode[nodeTId];
		if (nodeT.maxLeafDistance==0) return false;										// it's a leaf
		const _ui nodeCId = nodeT.childId[childId];
		const Node& nodeC = _pNode[nodeCId];
		const _ui axis = axisIndex(nodeT.level);
		const type avgT = getAxis(nodeT.elem.avg,       axis);
		const type avgL = getAxis(nodeC.elem.aabbAvg.l, axis);
		const type avgH = getAxis(nodeC.elem.aabbAvg.h, axis);
		return Math::bound<type>(avgT, avgL+s_avgCoordTolerance, avgH-s_avgCoordTolerance);
	}



	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::addLeaf(const Elem& elem, _ui nodeId, _ui childId)
	{
		//
		//    R           R
		//    |           |
		//    T    ->     P
		//               / \
		//              N   T
		//

		if (_numNodes+2>=_numNodesMax) return _numNodesMax;								// no additional nodes available
		// create new structure
		const _ui nodePId = addNodeRaw();
		const _ui nodeNId = addNodeRaw();
		const _ui nodeTId = nodeId;
		const _ui childNId =  childId;
		const _ui childTId = (childId+1) % s_numChildren;
		Node& nodeP = _pNode[nodePId];
		Node& nodeN = _pNode[nodeNId];
		Node& nodeT = _pNode[nodeTId];
		// update P
		nodeP.childId[childTId]	= nodeTId;
		nodeP.childId[childNId]	= nodeNId;
		nodeP.parentId			= nodeT.parentId;
		nodeP.parentChildId		= nodeT.parentChildId;
		nodeP.level				= nodeT.level;
		nodeP.maxLeafDistance	= 1;
		// update N
		nodeN.elem				= elem;
		nodeN.elem.aabbAvg		= elem.avg;
		nodeN.childId[0]		= _numNodesMax;
		nodeN.childId[1]		= _numNodesMax;
		nodeN.parentId			= nodePId;
		nodeN.parentChildId		= childNId;
		nodeN.level				= nodeP.level+1;
		nodeN.maxLeafDistance	= 0;
		nodeN.numAvg			= 1;
		// update T
		nodeT.parentId			= nodePId;
		nodeT.parentChildId		= childTId;
		nodeT.level				= nodeP.level+1;
		nodeT.maxLeafDistance	= 0;
		// update R
		updateParent(nodePId);
		// update root
		updateRoot(nodePId);
		// recalculate branch P
		recalculate(nodePId);
		// restructurize branch P
		restructurize(nodePId, _rootNodeId, 0);
		return nodeNId;
	}
	
	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::delLeaf(_ui nodeId)
	{
		//
		//      R          R
		//      |          |
		//      P    ->    N
		//     / \
		//    T   N
		//

		const _ui nodeTId = nodeId;
		const _ui nodeNId = getNeighborNode(nodeTId);
		Node& nodeT = _pNode[nodeTId];
		Node& nodeN = _pNode[nodeNId];
		const _ui nodePId = nodeT.parentId;
		Node& nodeP = _pNode[nodePId];
		// update N
		nodeN.parentId		= nodeP.parentId;
		nodeN.parentChildId	= nodeP.parentChildId;
		nodeN.level			= nodeP.level;
		// update R
		updateParent(nodeNId);
		// update N
		updateChildren(nodeNId);
		// update root
		updateRoot(nodeNId);
		// delete T and P
		delNodeRaw(nodePId);
		delNodeRaw(nodeTId);
		// recalculate branch N
		recalculate(nodeN.parentId);
		// restructurize branch N
		restructurize(nodeN.parentId, _rootNodeId, 1);
		return true;	
	}
	
	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::moveLeaf(_ui nodeFromId, _ui nodeToId)
	{
		//
		//      R          R               Q          Q
		//      |          |               |          |
		//      P    ->    X               T    ->    P
		//     / \              ------>              / \
		//    F   X                                 F   T
		//

		if (nodeFromId==nodeToId) return true;
		const _ui nodeFId = nodeFromId;
		const _ui nodeTId = nodeToId;
		Node& nodeF = _pNode[nodeFId];
		Node& nodeT = _pNode[nodeTId];
		const _ui nodePId = nodeF.parentId;
		const _ui nodeXId = getNeighborNode(nodeFId);
		Node& nodeP = _pNode[nodePId];
		Node& nodeX = _pNode[nodeXId];
		// update X
		nodeX.parentId			= nodeP.parentId;
		nodeX.parentChildId		= nodeP.parentChildId;
		nodeX.level				= nodeP.level;
		// update P
		const _ui childFId		= switchAxis(nodeF.elem.avg, nodeT.elem.avg, nodeT.level);
		const _ui childTId		= (childFId + 1) % s_numChildren;
		nodeP.childId[childFId] = nodeFId;
		nodeP.childId[childTId]	= nodeTId;
		nodeP.parentId			= nodeT.parentId;
		nodeP.parentChildId		= nodeT.parentChildId;
		nodeP.level				= nodeT.level;
		// update T
		nodeT.parentId			= nodePId;
		nodeT.parentChildId		= childTId;
		nodeT.level				= nodeP.level+1;
		// update F
		nodeF.parentId			= nodePId;
		nodeF.parentChildId		= childFId;
		nodeF.level				= nodeP.level+1;
		// update R
		updateParent(nodeXId);
		// update Q
		updateParent(nodePId);
		// update X
		updateChildren(nodeXId);
		// update root
		updateRoot(nodeXId);
		updateRoot(nodePId);
		// recalculate branch X
		recalculate(nodeX.parentId);
		// recalculate branch P
		recalculate(nodePId);
		return true;
	}



	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::updateElem(_ui nodeId)
	{
		Node& nodeT = _pNode[nodeId];
		const _ui nodeLId = nodeT.childId[s_childLId];
		const _ui nodeRId = nodeT.childId[s_childRId];
		const Node& nodeL = _pNode[nodeLId];
		const Node& nodeR = _pNode[nodeRId];
		nodeT.elem.avg		= ( (nodeL.elem.avg * (const type)(nodeL.numAvg))  +  (nodeR.elem.avg * (const type)(nodeR.numAvg)) )  /  (const type)(nodeT.numAvg);
		nodeT.elem.aabb		= nodeL.elem.aabb + nodeR.elem.aabb;
		nodeT.elem.aabbAvg	= nodeL.elem.aabbAvg + nodeR.elem.aabbAvg;
	}

	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::updateNode(_ui nodeId)
	{
		Node& nodeT = _pNode[nodeId];
		const _ui nodeLId = nodeT.childId[s_childLId];
		const _ui nodeRId = nodeT.childId[s_childRId];
		const Node& nodeL = _pNode[nodeLId];
		const Node& nodeR = _pNode[nodeRId];
		nodeT.numAvg			= nodeL.numAvg  + nodeR.numAvg;
		nodeT.maxLeafDistance	= Math::max<_ui>(nodeL.maxLeafDistance, nodeR.maxLeafDistance)+1;
		updateElem(nodeId);
	}



	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::updateRoot(_ui nodeId)
	{
		const _ui nodeTId = nodeId;
		const Node& nodeT = _pNode[nodeTId];
		if (nodeT.level!=0) return;
		_rootNodeId = nodeId;
	}

	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::updateParent(_ui nodeId)
	{
		const _ui nodeTId = nodeId;
		const Node& nodeT = _pNode[nodeTId];
		const _ui nodePId = _pNode[nodeTId].parentId;
		if (nodePId>=_numNodes) return;
		Node& nodeP = _pNode[nodePId];
		nodeP.childId[nodeT.parentChildId] = nodeTId;
	}

	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::updateChildren(_ui nodeId)
	{
		if (nodeId>=_numNodes) return;
		const _ui nodeTId = nodeId;
		Node& nodeT = _pNode[nodeTId];
		const _ui nodePId = nodeT.parentId;
		if (nodePId<_numNodes) nodeT.level = _pNode[nodePId].level+1;
		updateChildren(nodeT.childId[s_childLId]);
		updateChildren(nodeT.childId[s_childRId]);
	}

	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::recalculate(_ui nodeId)
	{
		if (nodeId>=_numNodes) return false;
		_ui nodeTId = nodeId;
		while (nodeTId<_numNodes)
		{
			Node& nodeT = _pNode[nodeTId];
			updateNode(nodeTId);
			nodeTId = nodeT.parentId;
		}
		return true;
	}

	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::restructurize(_ui nodeId, _ui rootNodeId, _ui childSwap)
	{
		//
		//          P                  P
		//         / \                / \
		//        CF CT              RF RT
		//       .     .            .     .
		//      .       .   --->   .       .
		//     PF       To        NF       PF
		//    /  \               /  \     /  \
		//   Fr  NF                      Fr  To
		//      /  \
		//

		if (nodeId>=_numNodes) return false;
		_ui nodeTId = nodeId;
		while (nodeTId!=rootNodeId)
		{
			Node& nodeT = _pNode[nodeTId];
			const _ui nodePId = nodeT.parentId;
			const _ui childId = (nodeT.parentChildId+childSwap) % s_numChildren;
			while (avgIntersection(nodePId, childId))
			{
				const Node& nodeP = _pNode[nodePId];
				const _ui axis = axisIndex(nodeP.level);
				const type avg = getAxis(nodeP.elem.avg, axis);
				const type sign = childId==s_childLId ? (const type)-1 : (const type)+1;
				const _ui childFId = childId;
				const _ui childTId = (childId+1) % s_numChildren;
				const _ui nodeCFId = nodeP.childId[childFId];
				const _ui nodeCTId = nodeP.childId[childTId];
				const _ui nodeFrId = searchOutLeaf(	nodeCFId, axis, avg, sign);
				const _ui nodeToId = searchLeaf(	nodeCTId, _pNode[nodeFrId].elem);
				const _ui nodeNFId = getNeighborNode(nodeFrId);
				const _ui nodePFId = _pNode[nodeFrId].parentId;
				moveLeaf(nodeFrId, nodeToId);
				const _ui nodeRFId = nodeP.childId[childFId];
				const _ui nodeRTId = nodeP.childId[childTId];
				if (!restructurize(nodeNFId, nodeRFId, 1)) return false;
				if (!restructurize(nodePFId, nodeRTId, 0)) return false;
			}
			nodeTId = nodePId;
		}
		return true;
	}



	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::sortX(_ui iLo, _ui iHi)
	{
		if (iLo>=iHi) return;
		register _ui lo = iLo;
		register _ui hi = iHi;
		register _ui mid = (lo + hi)>>1;
		const type val = _pNode[mid].elem.avg.x;
		do
		{
			while (_pNode[lo].elem.avg.x<=val && lo<mid) lo++;
			while (_pNode[hi].elem.avg.x>=val && hi>mid) hi--;
			if (lo<hi) __swap<Node>(_pNode[lo], _pNode[hi]);
		} while ((lo^mid) & (hi^mid));
		if (hi>iLo) sortX(iLo, hi-1);
		if (lo<iHi) sortX(lo+1, iHi);
	}

	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::sortY(_ui iLo, _ui iHi)
	{
		if (iLo>=iHi) return;
		register _ui lo = iLo;
		register _ui hi = iHi;
		register _ui mid = (lo + hi)>>1;
		const type val = _pNode[mid].elem.avg.y;
		do
		{
			while (_pNode[lo].elem.avg.y<=val && lo<mid) lo++;
			while (_pNode[hi].elem.avg.y>=val && hi>mid) hi--;
			if (lo<hi) __swap<Node>(_pNode[lo], _pNode[hi]);
		} while ((lo^mid) & (hi^mid));
		if (hi>iLo) sortY(iLo, hi-1);
		if (lo<iHi) sortY(lo+1, iHi);
	}

	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::sortZ(_ui iLo, _ui iHi)
	{
		if (iLo>=iHi) return;
		register _ui lo = iLo;
		register _ui hi = iHi;
		register _ui mid = (lo + hi)>>1;
		const type val = _pNode[mid].elem.avg.z;
		do
		{
			while (_pNode[lo].elem.avg.z<=val && lo<mid) lo++;
			while (_pNode[hi].elem.avg.z>=val && hi>mid) hi--;
			if (lo<hi) __swap<Node>(_pNode[lo], _pNode[hi]);
		} while ((lo^mid) & (hi^mid));
		if (hi>iLo) sortZ(iLo, hi-1);
		if (lo<iHi) sortZ(lo+1, iHi);
	}



	template <class callBack, typename type, typename data> _ui __fastcall BVH3<callBack, type, data>::sort(_ui iLo, _ui iHi, _ui level)
	{
		if (iLo==iHi) return iLo;
		if (_numNodes>=_numNodesMax) return _numNodesMax;
		const _ui i = level % 3;
		switch (i)
		{
			case 0 : sortX(iLo, iHi); break;
			case 1 : sortY(iLo, iHi); break;
			case 2 : sortZ(iLo, iHi); break;
		}
		const _ui iMid = (iLo + iHi)>>1;
		const _ui nodeLId = sort(iLo,    iMid, level+1);	if (nodeLId>=_numNodesMax) return _numNodesMax;
		const _ui nodeRId = sort(iMid+1, iHi,  level+1);	if (nodeRId>=_numNodesMax) return _numNodesMax;
		const _ui nodeTId = _numNodes;
		Node& nodeL = _pNode[nodeLId];
		Node& nodeR = _pNode[nodeRId];
		Node& nodeT = _pNode[nodeTId];
		nodeL.parentId = nodeTId;
		nodeR.parentId = nodeTId;
		nodeT.childId[s_childLId] = nodeLId;
		nodeT.childId[s_childRId] = nodeRId;
		nodeT.level = level;
		updateNode(nodeTId);
		_numNodes++;
		return nodeTId;
	}


	
	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::swapLeaf(_ui iFrom, _ui iTo)
	{
		Node& nodeF  = _pNode[iFrom];
		Node& nodeT  = _pNode[iTo];
		__swap<Node>(nodeF, nodeT);
		updateParent(iFrom);
		updateParent(iTo);
	}
	
	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::sortByUid(_ui iLo, _ui iHi)
	{
		if (iLo>=iHi) return;
		register _ui lo = iLo;
		register _ui hi = iHi;
		register _ui mid = (lo + hi)>>1;
		const _ui val = _pNode[mid].uid;
		do
		{
			while (_pNode[lo].uid<=val && lo<mid) lo++;
			while (_pNode[hi].uid>=val && hi>mid) hi--;
			if (lo<hi) swapLeaf(lo, hi);
		} while ((lo^mid) & (hi^mid));
		if (hi>iLo) sortByUid(iLo, hi-1);
		if (lo<iHi) sortByUid(lo+1, iHi);
	}
	
	
	
	template <class callBack, typename type, typename data> void __fastcall BVH3<callBack, type, data>::update(_ui nodeId)
	{
		Node& nodeT = _pNode[nodeId];
		if (nodeT.maxLeafDistance!=0)		// branch node
		{
			const _ui nodeLId = nodeT.childId[s_childLId];
			const _ui nodeRId = nodeT.childId[s_childRId];
			update(nodeLId);
			update(nodeRId);
			updateElem(nodeId);
		}
	}

	
	
	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::intersection(const Node& node, _ui nodeId) const
	{
		if (nodeId==_rootNodeId) return true;
		if (nodeId>=_numNodes) return false;
		const _ui nodeTId = nodeId;
		const Node& nodeT = _pNode[nodeTId];
		if (!node.aabb.intersect(nodeT.aabb)) return false;
		return true;
	}



	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::check(const Elem& elem, _ui nodeId, _b& bIntersection, _b bOneIntersection, _b bSubAvg, callBack& callBackClass, callBackIntersectionFunc intersectionFunc) const
	{
		if (bIntersection && bOneIntersection) return true;
		if (nodeId>=_numNodes) return false;
		const Node& node = _pNode[nodeId];
		if (bSubAvg && elem.aabbAvg.cover(node.elem.aabb)) return false;
		if (elem.aabb.intersect(node.elem.aabb))
		{
			if (node.maxLeafDistance!=0)		// branch node
			{
				check(elem, node.childId[s_childLId], bIntersection, bOneIntersection, bSubAvg, callBackClass, intersectionFunc);
				check(elem, node.childId[s_childRId], bIntersection, bOneIntersection, bSubAvg, callBackClass, intersectionFunc);
			}
			else								// leaf node
			{
				bIntersection |= (callBackClass.*intersectionFunc)(elem, node.elem);
			}
		}
		return bIntersection;
	}

	template <class callBack, typename type, typename data> _b __fastcall BVH3<callBack, type, data>::update(_ui nodeId, callBack& callBackClass, callBackUpdateFunc updateFunc)
	{
		if (nodeId>=_numNodes) return false;
		Node& node = _pNode[nodeId];
		if (node.maxLeafDistance!=0)		// branch node
		{
			update(node.childId[s_childLId], callBackClass, updateFunc);
			update(node.childId[s_childRId], callBackClass, updateFunc);
		}
		else								// leaf node
		{
			return (callBackClass.*updateFunc)(node.elem);
		}
		return true;
	}




};		// namespace Mpe

#endif	// __MPE_BVH3__
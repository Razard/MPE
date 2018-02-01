
// (c) Micelanholies 2015
// Micelanholies Physics Engine
// AVL - Adelson-Velsky and Landis tree based on indexes

#ifndef	__MPE_AVL__
#define	__MPE_AVL__

#include "MpeSimpleTypes.h"


namespace Mpe
{
	template <typename type, typename data> class AVL
	{
	public:
		struct Node
		{
			type	v;			// key value of node
			_ui		h;			// height of branch
			_ui		idP;		// index of parent node
			_ui		idL;		// index of left node
			_ui		idR;		// index of right node
			data	d;			// data of node
		};

	private:
		Node*	_pNode;
		_ui*	_pFreeNode;
		_ui		_numNodes;
		_ui		_numNodesMax;
		_ui		_numFreeNodes;
		_ui		_rootId;
		_ui		_minId;
		_ui		_maxId;

	public:
		AVL(void);
		AVL(_ui numNodesMax);
		~AVL(void);

		_b   __fastcall		init(_ui numNodesMax);
		void __fastcall		clear(void);

		_ui  __fastcall		numNodesMax(void) const;
		_ui  __fastcall		numNodes(void) const;
		_ui  __fastcall		numFreeNodes(void) const;

		_ui  __fastcall		rootId(void) const;								// index of root node
		_ui  __fastcall		minId(void) const;								// index of node with minimal key value
		_ui  __fastcall		maxId(void) const;								// index of node with maximal key value

		_ui  __fastcall		find(const type& v) const;						// seach node with value
		_ui  __fastcall		insert(const type& v, const data& d);			// insert new node with value and data
		_b	 __fastcall		remove(_ui nodeId);								// remove node

		_b   __fastcall		exist(_ui nodeId) const;						// check node exist
		_b   __fastcall		get(_ui nodeId, type& v, data& d) const;		// get key value and data pointer of node

		_ui  __fastcall		left(_ui nodeId) const;
		_ui  __fastcall		right(_ui nodeId) const;

		_b   __fastcall		verify(void) const;

	private:
		void __fastcall		reset(void);
		void __fastcall		flush(void);

		_ui  __fastcall		addNodeRaw(void);
		void __fastcall		delNodeRaw(_ui nodeId);

		_ui  __fastcall		height(_ui nodeId) const;						// height of node
		_i   __fastcall		delta(_ui nodeId) const;						// divergence between children branches heights of node
		void __fastcall		fix(_ui nodeId);								// fix height of node
		_ui  __fastcall		rotateR(_ui nodeId);							// right rotate around node
		_ui  __fastcall		rotateL(_ui nodeId);							// left rotate around node
		void __fastcall		update(_ui nodeId, _ui idF, _ui idT);			// update one of node's child from idF to idT
		void __fastcall		balance(_ui nodeId);							// balance from node to root
		_ui  __fastcall		findMin(_ui nodeId) const;						// search node with minimal value in tree
		_ui  __fastcall		findMax(_ui nodeId) const;						// search node with maximal value in tree

	};	// class AVL


	template <typename type, typename data> AVL<type,data>::AVL(void)
	{
		reset();
	}

	template <typename type, typename data> AVL<type,data>::AVL(_ui numNodesMax)
	{
		init(numNodesMax);
	}

	template <typename type, typename data> AVL<type,data>::~AVL(void)
	{
		flush();
	}



	template <typename type, typename data> _b __fastcall AVL<type,data>::init(_ui numNodesMax)
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
		_numNodesMax	= numNodesMax;
		_numFreeNodes	= 0;
		return true;
	}

	template <typename type, typename data> void __fastcall AVL<type,data>::clear(void)
	{
		_numNodes		= 0;
		_numFreeNodes	= 0;
		_rootId			= 0;
		_minId			= 0;
		_maxId			= 0;
	}



	template <typename type, typename data> _ui __fastcall AVL<type,data>::numNodesMax(void) const
	{
		return _numNodesMax;
	}

	template <typename type, typename data> _ui __fastcall AVL<type,data>::numNodes(void) const
	{
		return _numNodes;
	}

	template <typename type, typename data> _ui __fastcall AVL<type,data>::numFreeNodes(void) const
	{
		return _numFreeNodes;
	}

	

	template <typename type, typename data> _ui __fastcall AVL<type,data>::rootId(void) const
	{
		return _rootId;
	}
	
	template <typename type, typename data> _ui __fastcall AVL<type,data>::minId(void) const
	{
		return _minId;
	}
	
	template <typename type, typename data> _ui __fastcall AVL<type,data>::maxId(void) const
	{
		return _maxId;
	}



	template <typename type, typename data> _ui __fastcall AVL<type,data>::find(const type& v) const
	{
		_ui nodePId = _numNodes;
		_ui nodeTId = _rootId;
		while (nodeTId<_numNodes)
		{
			const Node& node = _pNode[nodeTId];
			nodePId = nodeTId;
			nodeTId = (v < node.v) ? node.idL : node.idR;
			if (nodeTId>=_numNodes) return nodePId;
		}
		return _numNodesMax;
	}

	template <typename type, typename data> _ui __fastcall AVL<type,data>::insert(const type& v, const data& d)
	{
		//      
		//      |    v < P.v     |
		//      P    ------>     P
		//       \              / \
		//        R            N   R
		//
		//
		//      |    v >= P.v    |
		//      P    ------->    P
		//     /                / \
		//    L                L   N
		//

		const _ui nodePId = find(v);
		const _ui nodeNId = addNodeRaw();
		if (nodeNId>=_numNodesMax) return _numNodesMax;
		Node& nodeN = _pNode[nodeNId];
		nodeN.v		= v;
		nodeN.h		= 1;
		nodeN.idP	= nodePId;
		nodeN.idL	= _numNodesMax;
		nodeN.idR	= _numNodesMax;
		nodeN.d		= d;
		if (nodePId<_numNodes)
		{
			Node& nodeP = _pNode[nodePId];
			if (v < nodeP.v)	nodeP.idL = nodeNId;
			else				nodeP.idR = nodeNId;
		}
		if (nodePId==_minId && v < _pNode[_minId].v)	_minId = nodeNId;
		if (nodePId==_maxId && v >=_pNode[_maxId].v)	_maxId = nodeNId;
		balance(nodeNId);
		return nodeNId;
	}

	template <typename type, typename data> _b __fastcall AVL<type,data>::remove(_ui nodeId)
	{
		//
		//         P                 P
		//         |                 |
		//         T       -->       M
		//        / \               / \
		//       /   \             /   \
		//      /     \           /     \
		//     /       \         /       \
		//    /         \       /         \
		//   /           \     /           \
		//  L             R   L             R
		//               /                 /
		//              .                 .
		//             /                 /
		//            N                 N
		//           /                 /
		//          M                 K
		//           \
		//            K
		//
		// L <= T <= M <= K <= N <= R
		//

		if (!exist(nodeId)) return false;
		if (_pNode[nodeId].h>=_numNodesMax) return false;
		const _ui nodeTId = nodeId;
		const Node nodeT = _pNode[nodeTId];
		const _ui nodePId = nodeT.idP;
		const _ui nodeLId = nodeT.idL;
		const _ui nodeRId = nodeT.idR;
		if (nodeTId==_maxId) _maxId = left(nodeTId);
		if (nodeTId==_minId) _minId = right(nodeTId);
		delNodeRaw(nodeTId);
		if (nodeRId>=_numNodes)
		{
			if (nodeTId==_rootId) _rootId = nodeLId;
			update(nodePId, nodeTId, nodeLId);
			balance(nodeLId);
			return true;
		}
		const _ui nodeMId = findMin(nodeRId);
		Node& nodeM = _pNode[nodeMId];
		if (nodeMId==nodeRId)
		{
			if (nodeTId==_rootId) _rootId = nodeRId;
			nodeM.idL = nodeLId;
			nodeM.idP = nodePId;
			if (exist(nodeLId)) _pNode[nodeLId].idP = nodeMId;
			update(nodePId, nodeTId, nodeMId);
			return true;
		}
		const _ui nodeNId = nodeM.idP;
		Node& nodeN = _pNode[nodeNId];
		const _ui nodeKId = nodeM.idR;
		if (exist(nodeKId)) _pNode[nodeKId].idP = nodeNId;
		if (exist(nodeLId)) _pNode[nodeLId].idP = nodeMId;
		if (exist(nodeRId)) _pNode[nodeRId].idP = nodeMId;
		nodeM.idP = nodePId;
		nodeM.idL = nodeMId!=nodeLId ? nodeT.idL : _numNodesMax;
		nodeM.idR = nodeMId!=nodeRId ? nodeT.idR : _numNodesMax;
		update(nodeNId, nodeMId, nodeKId);
		update(nodePId, nodeTId, nodeMId);
		if (nodeTId==_rootId) _rootId = nodeMId;
		balance(nodeNId);
		return true;
	}



	template <typename type, typename data> _b __fastcall AVL<type,data>::exist(_ui nodeId) const
	{
		if (nodeId>=_numNodes) return false;
		if (_pNode[nodeId].h>=_numNodesMax) return false;
		return true;
	}
	
	template <typename type, typename data> _b __fastcall AVL<type,data>::get(_ui nodeId, type& v, data& d) const
	{
		if (!exist(nodeId)) return false;
		const Node& node = _pNode[nodeId];
		v = node.v;
		d = node.d;
		return true;
	}



	template <typename type, typename data> _ui __fastcall AVL<type,data>::left(_ui nodeId) const
	{
		if (!exist(nodeId)) return _numNodesMax;
		const Node& node = _pNode[nodeId];
		if (exist(node.idL)) return findMax(node.idL);
		_ui nodeTId = nodeId;
		_ui nodePId = node.idP;
		while (nodePId<_numNodes)
		{
			if (_pNode[nodePId].idR==nodeTId) return nodePId;
			nodeTId = nodePId;
			nodePId = _pNode[nodeTId].idP;
		}
		return _numNodesMax;
	}
	
	template <typename type, typename data> _ui __fastcall AVL<type,data>::right(_ui nodeId) const
	{
		if (!exist(nodeId)) return _numNodesMax;
		const Node& node = _pNode[nodeId];
		if (exist(node.idR)) return findMin(node.idR);
		_ui nodeTId = nodeId;
		_ui nodePId = node.idP;
		while (nodePId<_numNodes)
		{
			if (_pNode[nodePId].idL==nodeTId) return nodePId;
			nodeTId = nodePId;
			nodePId = _pNode[nodeTId].idP;
		}
		return _numNodesMax;
	}



	template <typename type, typename data> _b __fastcall AVL<type,data>::verify(void) const
	{
		for (_ui nodeId = 0; nodeId<_numNodes; nodeId++)
		{
			if (!exist(nodeId)) continue;
			const _ui nodeTId = nodeId;
			const Node& nodeT = _pNode[nodeTId];
			const _ui nodePId = nodeT.idP;
			const _ui nodeLId = nodeT.idL;
			const _ui nodeRId = nodeT.idR;
			if (nodePId<_numNodes && !exist(nodePId))
			{
				return false;
			}
			if (nodeLId<_numNodes && !exist(nodeLId))
			{
				return false;
			}
			if (nodeRId<_numNodes && !exist(nodeRId))
			{
				return false;
			}
			if (exist(nodePId))
			{
				const Node& nodeP = _pNode[nodePId];
				if (nodeP.idL!=nodeTId && nodeP.idR!=nodeTId)
				{
					return false;
				}
			}
			if (exist(nodeLId))
			{
				const Node& nodeL = _pNode[nodeLId];
				if (nodeL.idP!=nodeTId)
				{
					return false;
				}
			}
			if (exist(nodeRId))
			{
				const Node& nodeR = _pNode[nodeRId];
				if (nodeR.idP!=nodeTId)
				{
					return false;
				}
			}
		}
		return true;
	}



	template <typename type, typename data> void __fastcall AVL<type,data>::reset(void)
	{
		_pNode			= NULL;
		_pFreeNode		= NULL;
		_numNodesMax	= 0;
		clear();
	}
	
	template <typename type, typename data> void __fastcall AVL<type,data>::flush(void)
	{
		try	{	delete[] _pNode;		}	catch(...)	{};
		try	{	delete[] _pFreeNode;	}	catch(...)	{};
		reset();
	}



	template <typename type, typename data> _ui __fastcall AVL<type,data>::addNodeRaw(void)
	{
		return _numFreeNodes ? _pFreeNode[--_numFreeNodes] : _numNodes++;
	}
	
	template <typename type, typename data> void __fastcall AVL<type,data>::delNodeRaw(_ui nodeId)
	{
		if (!exist(nodeId)) return;
		if (nodeId<_numNodes-1)	_pFreeNode[_numFreeNodes++] = nodeId;
		else					_numNodes--;
		_pNode[nodeId].h = _numNodesMax;
	}



	template <typename type, typename data> _ui __fastcall AVL<type,data>::height(_ui nodeId) const
	{
		return nodeId<_numNodes ? _pNode[nodeId].h : 0;
	}
	
	template <typename type, typename data> _i __fastcall AVL<type,data>::delta(_ui nodeId) const
	{
		return height(_pNode[nodeId].idR) - height(_pNode[nodeId].idL);
	}
	
	template <typename type, typename data> void __fastcall AVL<type,data>::fix(_ui nodeId)
	{
		Node& node = _pNode[nodeId];
		const _ui hL = height(node.idL);
		const _ui hR = height(node.idR);
		node.h = (hL>hR ? hL : hR) + 1;
	}
	
	template <typename type, typename data> _ui __fastcall AVL<type,data>::rotateR(_ui nodeId)
	{
		//      P          P
		//      |          |
		//      X          Y
		//     / \   ->   / \
		//    Y   C      A   X
		//   / \            / \
		//  A   B          B   C

		const _ui idX = nodeId;
		const _ui idY = _pNode[idX].idL;
		const _ui idP = _pNode[idX].idP;
		const _ui idA = _pNode[idY].idL;
		const _ui idB = _pNode[idY].idR;
		const _ui idC = _pNode[idX].idR;
		_pNode[idX].idP = idY;
		_pNode[idX].idL = idB;
		_pNode[idY].idP = idP;
		_pNode[idY].idR = idX;
		if (idB<_numNodes) _pNode[idB].idP = idX;
		update(idP, idX, idY);
		fix(idX);
		fix(idY);
		if (_rootId==idX) _rootId = idY;
		return idY;
	}
	
	template <typename type, typename data> _ui __fastcall AVL<type,data>::rotateL(_ui nodeId)
	{
		//    P            P
		//    |            |
		//    X            Y
		//   / \    ->    / \
		//  A   Y        X   C
		//     / \      / \
		//    B   C    A   B

		const _ui idX = nodeId;
		const _ui idY = _pNode[idX].idR;
		const _ui idP = _pNode[idX].idP;
		const _ui idA = _pNode[idX].idL;
		const _ui idB = _pNode[idY].idL;
		const _ui idC = _pNode[idY].idR;
		_pNode[idX].idP = idY;
		_pNode[idX].idR = idB;
		_pNode[idY].idP = idP;
		_pNode[idY].idL = idX;
		if (idB<_numNodes) _pNode[idB].idP = idX;
		update(idP, idX, idY);
		fix(idX);
		fix(idY);
		if (_rootId==idX) _rootId = idY;
		return idY;
	}

	template <typename type, typename data> void __fastcall AVL<type,data>::update(_ui nodeId, _ui idF, _ui idT)
	{
		if (exist(idT))		_pNode[idT].idP = nodeId;
		if (!exist(nodeId)) return;
		Node& node = _pNode[nodeId];
		if (node.idL==idF)	node.idL = idT;
		else				node.idR = idT;
	}

	
	template <typename type, typename data> void __fastcall AVL<type,data>::balance(_ui nodeId)
	{
		if (!exist(nodeId)) return;
		_ui nodePId = _numNodes;
		_ui nodeTId = nodeId;
		while (nodeTId<_numNodes)
		{
			Node& nodeT = _pNode[nodeTId];
			nodePId = nodeT.idP;
			fix(nodeTId);
			if (delta(nodeTId)==2)
			{
				if (delta(nodeT.idR)<0) rotateR(nodeT.idR);
				rotateL(nodeTId);
			}
			if (delta(nodeTId)==-2)
			{
				if (delta(nodeT.idL)>0) rotateL(nodeT.idL);
				rotateR(nodeTId);
			}
			nodeTId = nodePId;
		}
	}

	template <typename type, typename data> _ui __fastcall AVL<type,data>::findMin(_ui nodeId) const
	{
		if (!exist(nodeId)) return nodeId;
		while (_pNode[nodeId].idL<_numNodes)
			nodeId = _pNode[nodeId].idL;
		return nodeId;
	}
	
	template <typename type, typename data> _ui __fastcall AVL<type,data>::findMax(_ui nodeId) const
	{
		if (!exist(nodeId)) return nodeId;
		while (_pNode[nodeId].idR<_numNodes)
			nodeId = _pNode[nodeId].idR;
		return nodeId;
	}

};	// namespace Mpe

#endif	// __MPE_AVL__





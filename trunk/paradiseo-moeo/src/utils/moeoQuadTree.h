/*
* <moeoQuadTree.h>
* Copyright (C) DOLPHIN Project-Team, INRIA Futurs, 2006-2007
* (C) OPAC Team, LIFL, 2002-2007
*
* Arnaud Liefooghe
* Jérémie Humeau
*
* This software is governed by the CeCILL license under French law and
* abiding by the rules of distribution of free software.  You can  use,
* modify and/ or redistribute the software under the terms of the CeCILL
* license as circulated by CEA, CNRS and INRIA at the following URL
* "http://www.cecill.info".
*
* As a counterpart to the access to the source code and  rights to copy,
* modify and redistribute granted by the license, users are provided only
* with a limited warranty  and the software's author,  the holder of the
* economic rights,  and the successive licensors  have only  limited liability.
*
* In this respect, the user's attention is drawn to the risks associated
* with loading,  using,  modifying and/or developing or reproducing the
* software by the user in light of its specific status of free software,
* that may mean  that it is complicated to manipulate,  and  that  also
* therefore means  that it is reserved for developers  and  experienced
* professionals having in-depth computer knowledge. Users are therefore
* encouraged to load and test the software's suitability as regards their
* requirements in conditions enabling the security of their systems and/or
* data to be ensured and,  more generally, to use and operate it in the
* same conditions as regards security.
* The fact that you are presently reading this means that you have had
* knowledge of the CeCILL license and that you accept its terms.
*
* ParadisEO WebSite : http://paradiseo.gforge.inria.fr
* Contact: paradiseo-help@lists.gforge.inria.fr
*
*/
//-----------------------------------------------------------------------------

#ifndef MOEOQUADTREE_H_
#define MOEOQUADTREE_H_

template < class ObjectiveVector >
class QuadTreeNode{
public:
	QuadTreeNode(ObjectiveVector& _objVec):objVec(_objVec),subTree(){}

	QuadTreeNode(const QuadTreeNode& _source):objVec(_source.objVec),subTree(_source.subTree){}

	QuadTreeNode& operator=(const QuadTreeNode& _src){
		(*this).objVec=_src.objVec;
		(*this).subTree=subTree;
		return *this;
	}

	ObjectiveVector& getVec(){
		return objVec;
	}

	bool setChild(unsigned int _kSuccesor, QuadTreeNode<ObjectiveVector>* _child){
		bool res = false;
		if((*this).subTree[_kSuccesor] != NULL)
			res=true;
		else{
			(*this).subTree[_kSuccesor]= _child;
			std::cout <<"setChild: " <<  getVec() << std::endl;
		}
		return res;
	}

	std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>& getSubTree(){
		return (*this).subTree;
	}

private:
	ObjectiveVector& objVec;
	std::map<unsigned int, QuadTreeNode<ObjectiveVector>*> subTree;
};



template < class ObjectiveVector >
class moeoQuadTree{
public:
	moeoQuadTree():root(NULL){
		bound=pow(2,ObjectiveVector::nObjectives())-1;
	}

	bool insert(ObjectiveVector& _obj){
		bool res=false;
		bool stop=false;
		QuadTreeNode<ObjectiveVector>* tmp = new QuadTreeNode<ObjectiveVector>(_obj);
		QuadTreeNode<ObjectiveVector>* realroot = root;
		//the tree is empty -> create a node and fix it at the root
		if(isEmpty()){
			root=tmp;
			res=true;
			std::cout << "insert case empty: " << root->getVec() << std::endl;
		}
		else{
			while(!stop){
				//calulate the k-Successor de _obj wtih respect to the root
				unsigned int succ=k_succ(_obj, root->getVec());
				if(succ != bound){
					if(succ == 0){
						replace(_obj);
						res=true;
					}
					else{
						//dominance test1
						typename std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>::iterator it=root->getSubTree().begin();
						while(!stop && (it != root->getSubTree().end())){
							if( ((*it).first < succ) && (((succ ^ bound) & ((*it).first ^ bound))== succ) ){
								stop = test1(tmp, (*it).second);
							}
							it++;
						}
						if(!stop){
							//dominance test2
							typename std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>::iterator it=root->getSubTree().begin();
							while(it != root->getSubTree().end()){
								if( (succ < (*it).first) && ((succ & (*it).first) == succ)){
									test2(tmp, (*it).second, root, (*it).first);
								}
								it++;
							}
							//insertion
							QuadTreeNode<ObjectiveVector>* tmp = new QuadTreeNode<ObjectiveVector>(_obj);
							std::cout << "insert case new son: " << root->getVec() << std::endl;
							if(root->setChild(succ, tmp)){
								root=root->getSubTree()[succ];
							}
							else{
								res=true;
								stop=true;
							}
						}
					}
				}
			}
		}
		root=realroot;
		return res;
	}

	/*
	 * return the k-successor of _objVec1 with respect to _objVec2
	 * @param _objVec1
	 * @param _objVec2
	 */
	unsigned int k_succ(const ObjectiveVector& _objVec1, const ObjectiveVector& _objVec2){
		unsigned int res=0;
		for(int i=0; i < ObjectiveVector::nObjectives(); i++){
			if( (ObjectiveVector::minimizing(i) && ((_objVec1[i] - _objVec2[i]) >= (-1.0 * 1e-6 ))) ||
				(ObjectiveVector::maximizing(i) && ((_objVec1[i] - _objVec2[i]) <= 1e-6 ))){
				res+=pow(2,ObjectiveVector::nObjectives()-i-1);
			}
		}
		return res;
	}

	/*
	 * replace the old root by the new one
	 * @param _newroot
	 */
	void replace(ObjectiveVector _newroot){
		//create the new root
		QuadTreeNode<ObjectiveVector>* newroot = new QuadTreeNode<ObjectiveVector>(_newroot);
		//reconsider each son of the old root
		if(!(root->getSubTree().empty())){
			typename std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>::iterator it;
			for(it=(root->getSubTree()).begin(); it != (root->getSubTree()).end(); it++){
				std::cout << "replace: " << (*it).second->getVec() << std::endl;
				reconsider(newroot, (*it).second);
			}
		}
		//replace the old root by the new one
		delete(root);
		root = newroot;
	}

	void reconsider(QuadTreeNode<ObjectiveVector>* _newroot, QuadTreeNode<ObjectiveVector>* _child){
		unsigned int succ;
		std::cout << "reconsider: " << _child->getVec() << std::endl;
		if(!(_child->getSubTree().empty())){
			std::cout << "enter reconsider" << std::endl;
			typename std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>::iterator it;
			for(it=(_child->getSubTree()).begin(); it != (_child->getSubTree()).end(); it++){
				std::cout << "reconsider: " << (*it).second->getVec() << std::endl;
				QuadTreeNode<ObjectiveVector>* tmp=(*it).second;
				_child->getSubTree().erase(it);
				reconsider(_newroot, tmp);
			}
		}
		else{
			std::cout << "reconsider: no more child" << std::endl;
		}
		succ=k_succ(_child->getVec(),_newroot->getVec());
		if(succ==bound)
			delete(_child);
		else if(_newroot->getSubTree()[succ] != NULL){
			reinsert(_newroot->getSubTree()[succ],_child);
		}
		else{
			_newroot->setChild(succ, _child);
		}
	}

	void reinsert(QuadTreeNode<ObjectiveVector>* _node1, QuadTreeNode<ObjectiveVector>* _node2){
		unsigned int succ;
		if(!(_node1->getSubTree().empty())){
			typename std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>::iterator it;
			for(it=(_node1->getSubTree()).begin(); it != (_node1->getSubTree()).end(); it++){
				std::cout << "reinsert: " << (*it).second->getVec() << std::endl;
				QuadTreeNode<ObjectiveVector>* tmp=(*it).second;
				_node1->getSubTree().erase(it);
				reinsert(_node1, tmp);
			}
		}
		succ=k_succ(_node2->getVec(),_node1->getVec());
		if(_node1->getSubTree()[succ] != NULL){
			reinsert(_node1->getSubTree()[succ],_node2);
		}
		else{
			_node1->setChild(succ, _node2);
		}
	}

	void remove(QuadTreeNode<ObjectiveVector>* _node, QuadTreeNode<ObjectiveVector>* _parent, unsigned int _succ){
		unsigned int k=1;
		QuadTreeNode<ObjectiveVector>* tmp=NULL;
		_parent->getSubTree()[_succ]=NULL;
		while((k < (bound -1)) && _node->getSubTree()[k]==NULL){
			k++;
		}
		if(_node->getSubTree()[k]!=NULL){
			tmp =_node->getSubTree()[k];
			_parent->setChild(_succ, tmp);
		}
		k++;
		while(k < (bound -1)){
			if(_node->getSubTree()[k]!=NULL){
				reinsert(tmp ,_node->getSubTree()[k]);
			}
			k++;
		}
		delete(_node);
	}

	bool test1(QuadTreeNode<ObjectiveVector>* _node1, QuadTreeNode<ObjectiveVector>* _node2){
		bool res = false;
		unsigned int succ;
		succ=k_succ(_node1->getVec(), _node2->getVec());
		if(succ==bound){
			res=true;
		}
		else{
			typename std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>::iterator it=_node2->getSubTree().begin();
			while(!res && (it != _node2->getSubTree().end())){
				if( ((succ ^ bound) & ((*it).first ^ bound)) == succ){
					res = res || test1(_node1, (*it).second);
				}
				it++;
			}
		}
		return res;
	}

	void test2(QuadTreeNode<ObjectiveVector>* _node1, QuadTreeNode<ObjectiveVector>* _node2, QuadTreeNode<ObjectiveVector>* _parent, unsigned int _succ){
		unsigned int succ;
		succ=k_succ(_node1->getVec(), _node2->getVec());
		if(succ==0)
			remove(_node2, _parent, _succ);
		typename std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>::iterator it=_node2->getSubTree().begin();
		while(it != _node2->getSubTree().end()){
			if( (succ & (*it).first) == succ){
				test2(_node1, (*it).second, _node2, (*it).first);
			}
			it++;
		}
	}



	void printTree(){
//		typename std::map<unsigned int, QuadTreeNode<ObjectiveVector>*>::iterator it;
//		std::cout << "root: " << root->getVec() << "&" << std::endl << "childs:" << std::endl;
//		for(it=(root->getSubTree()).begin(); it != (root->getSubTree()).end(); ++it)
//			std::cout << (*it).second->getVec() << std::endl;
	}

	bool isEmpty(){
		return root==NULL;
	}

private:


	QuadTreeNode<ObjectiveVector>* root;
	unsigned int bound;
	std::list< QuadTreeNode<ObjectiveVector> > nodes;



};



#endif /*MOEOQUADTREE_H_*/

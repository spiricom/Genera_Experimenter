/*
 * AVLtree.h
 *
 *  Created on: Jul 28, 2020
 *      Author: josnyder
 */

#ifndef AVLTREE_H_
#define AVLTREE_H_

// An AVL tree node
struct AVLNode
{
    int key;
    int count;
    struct AVLNode *left;
    struct AVLNode *right;
    int height;
};
extern int AVL_memory_position;
extern int AVL_memory_offset;

int max(int a, int b);
int min(int a, int b);
int height(struct AVLNode *N);
struct AVLNode* newNode(int key);
struct AVLNode *rightRotate(struct AVLNode *y);
struct AVLNode *leftRotate(struct AVLNode *x);
int getBalance(struct AVLNode *N);
struct AVLNode* insert(struct AVLNode* node, int key);
struct AVLNode* deleteNode(struct AVLNode* root, int key);
void preOrder(struct AVLNode *root);
struct AVLNode * minValueNode(struct AVLNode* node);
struct AVLNode * maxValueNode(struct AVLNode* node);
int minValueKey(struct AVLNode* node);
int maxValueKey(struct AVLNode* node);

#endif /* AVLTREE_H_ */

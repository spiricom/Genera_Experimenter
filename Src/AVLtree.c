/*
 * AVLtree.c
 *
 *  Created on: Jul 28, 2020
 *      Author: josnyder
 */

// C program to insert a node in AVL tree
#include<stdio.h>
#include<stdlib.h>

#include "AVLtree.h"
#include "leaf.h"
#include "audiostream.h"

int AVL_memory_position = 0;
int AVL_memory_offset = 0;

int blocksUsed[512] = {0};

// A utility function to get the height of the tree
int height(struct AVLNode *N)
{
    if (N == NULL)
        return 0;
    return N->height;
}

// A utility function to get maximum of two integers
int max(int a, int b)
{
    return (a > b)? a : b;
}

// A utility function to get maximum of two integers
int min(int a, int b)
{
    return (a < b)? a : b;
}

/* Helper function that allocates a new node with the given key and
    NULL left and right pointers. */
struct AVLNode* newNode(int key)
{
    while (blocksUsed[AVL_memory_position] > 0)
    {
    	AVL_memory_position = (AVL_memory_position + 1) & 511;
    }

    struct AVLNode* node = (struct AVLNode*) (((AVL_memory_position * sizeof(struct AVLNode)) + AVL_memory_offset));
    blocksUsed[AVL_memory_position] = 1;
    //AVL_memory_position+= sizeof(struct AVLNode); //increment memory pointer past new node
		//mpool_calloc(sizeof(struct AVLNode), mediumPool);
    node->key   = key;
    node->left   = NULL;
    node->right  = NULL;
    node->count = 0;
    node->height = 1;  // new node is initially added at leaf
    return(node);
}

// A utility function to right rotate subtree rooted with y
// See the diagram given above.
struct AVLNode *rightRotate(struct AVLNode *y)
{
    struct AVLNode *x = y->left;
    struct AVLNode *T2 = x->right;

    // Perform rotation
    x->right = y;
    y->left = T2;

    // Update heights
    y->height = max(height(y->left), height(y->right))+1;
    x->height = max(height(x->left), height(x->right))+1;

    // Return new root
    return x;
}

// A utility function to left rotate subtree rooted with x
// See the diagram given above.
struct AVLNode *leftRotate(struct AVLNode *x)
{
    struct AVLNode *y = x->right;
    struct AVLNode *T2 = y->left;

    // Perform rotation
    y->left = x;
    x->right = T2;

    //  Update heights
    x->height = max(height(x->left), height(x->right))+1;
    y->height = max(height(y->left), height(y->right))+1;

    // Return new root
    return y;
}

// Get Balance factor of node N
int getBalance(struct AVLNode *N)
{
    if (N == NULL)
        return 0;
    return height(N->left) - height(N->right);
}

// Recursive function to insert a key in the subtree rooted
// with node and returns the new root of the subtree.
struct AVLNode* insert(struct AVLNode* node, int key)
{
    /* 1.  Perform the normal BST insertion */
    if (node == NULL)
        return(newNode(key));

    if (key < node->key)
        node->left  = insert(node->left, key);
    else if (key > node->key)
        node->right = insert(node->right, key);
    else // duplicate key -- increment count in this node
    {
        (node->count)++;
    	return node;
    }
    /* 2. Update height of this ancestor node */
    node->height = 1 + max(height(node->left),
                           height(node->right));

    /* 3. Get the balance factor of this ancestor
          node to check whether this node became
          unbalanced */
    int balance = getBalance(node);

    // If this node becomes unbalanced, then
    // there are 4 cases

    // Left Left Case
    if (balance > 1 && key < node->left->key)
        return rightRotate(node);

    // Right Right Case
    if (balance < -1 && key > node->right->key)
        return leftRotate(node);

    // Left Right Case
    if (balance > 1 && key > node->left->key)
    {
        node->left =  leftRotate(node->left);
        return rightRotate(node);
    }

    // Right Left Case
    if (balance < -1 && key < node->right->key)
    {
        node->right = rightRotate(node->right);
        return leftRotate(node);
    }

    /* return the (unchanged) node pointer */
    return node;
}
// Recursive function to delete a node with given key
// from subtree with given root. It returns root of
// the modified subtree.
struct AVLNode* deleteNode(struct AVLNode* root, int key)
{
    // STEP 1: PERFORM STANDARD BST DELETE

    if (root == NULL)
        return root;

    // If the key to be deleted is smaller than the
    // root's key, then it lies in left subtree
    if ( key < root->key )
        root->left = deleteNode(root->left, key);

    // If the key to be deleted is greater than the
    // root's key, then it lies in right subtree
    else if( key > root->key )
        root->right = deleteNode(root->right, key);

    // if key is same as root's key, then This is
    // the node to be deleted
    else if ( key == root->key)
    {
        if (root->count > 0)
        {
        	root->count--;
        	return root;
        }

    	// node with only one child or no child
        else if( (root->left == NULL) || (root->right == NULL) )
        {
            struct AVLNode *temp = root->left ? root->left :
                                             root->right;

            // No child case
            if (temp == NULL)
            {
                temp = root;
                root = NULL;
            }
            else // One child case
            {
             *root = *temp; // Copy the contents of
                            // the non-empty child
            }
            //mpool_free(temp, mediumPool);
            AVL_memory_position = ((int)temp - AVL_memory_offset) / sizeof(struct AVLNode); //set the memory pointer to use this memory next
            blocksUsed[AVL_memory_position] = 0;

        }
        else
        {
            // node with two children: Get the inorder
            // successor (smallest in the right subtree)
            struct AVLNode* temp = minValueNode(root->right);

            // Copy the inorder successor's data to this node
            root->key = temp->key;

            // Delete the inorder successor
            root->right = deleteNode(root->right, temp->key);
        }
    }

    else
    {
    	return root;
    }
    // If the tree had only one node then return
    if (root == NULL)
      return root;

    // STEP 2: UPDATE HEIGHT OF THE CURRENT NODE
    root->height = 1 + max(height(root->left),
                           height(root->right));

    // STEP 3: GET THE BALANCE FACTOR OF THIS NODE (to
    // check whether this node became unbalanced)
    int balance = getBalance(root);

    // If this node becomes unbalanced, then there are 4 cases

    // Left Left Case
    if (balance > 1 && getBalance(root->left) >= 0)
        return rightRotate(root);

    // Left Right Case
    if (balance > 1 && getBalance(root->left) < 0)
    {
        root->left =  leftRotate(root->left);
        return rightRotate(root);
    }

    // Right Right Case
    if (balance < -1 && getBalance(root->right) <= 0)
        return leftRotate(root);

    // Right Left Case
    if (balance < -1 && getBalance(root->right) > 0)
    {
        root->right = rightRotate(root->right);
        return leftRotate(root);
    }

    return root;
}
// A utility function to print preorder traversal
// of the tree.
// The function also prints height of every node
void preOrder(struct AVLNode *root)
{
    if(root != NULL)
    {
        printf("%d ", root->key);
        preOrder(root->left);
        preOrder(root->right);
    }
}

struct AVLNode * minValueNode(struct AVLNode* node)
{
    struct AVLNode* current = node;

    /* loop down to find the leftmost leaf */
    while (current->left != NULL)
        current = current->left;

    return current;
}

struct AVLNode * maxValueNode(struct AVLNode* node)
{
    struct AVLNode* current = node;

    /* loop down to find the leftmost leaf */
    while (current->right != NULL)
        current = current->right;

    return current;
}

int minValueKey(struct AVLNode* node)
{
    struct AVLNode* current = node;

    /* loop down to find the leftmost leaf */
    while (current->left != NULL)
        current = current->left;

    return current->key;
}


int maxValueKey(struct AVLNode* node)
{
    struct AVLNode* current = node;

    /* loop down to find the leftmost leaf */
    while (current->right != NULL)
        current = current->right;

    return current->key;
}
/*
int main()
{
  struct AVLNode *root = NULL;


  root = insert(root, 10);
  root = insert(root, 20);
  root = insert(root, 30);
  root = insert(root, 40);
  root = insert(root, 50);
  root = insert(root, 25);
  */
  /* The constructed AVL Tree would be
            30
           /  \
         20   40
        /  \     \
       10  25    50
  */

  //printf("Preorder traversal of the constructed AVL"
        // " tree is \n");
 // preOrder(root);

 // return 0;
//}

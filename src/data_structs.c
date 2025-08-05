#include "data_structs.h"
#include <stdlib.h>

TreeNode* tree_create_node(float data){
    TreeNode *node = malloc(sizeof(TreeNode));
    if (!node) exit(EXIT_FAILURE);
    node->data = data;
    node->left = NULL;
    node->right = NULL;
    return node;
}

TreeNode* tree_add_node_left(TreeNode *root, float data){
    root->left = tree_create_node(data);
    return root->left;
}

TreeNode* tree_add_node_right(TreeNode *root, float data){
    root->right = tree_create_node(data);
    return root->right;
}

void tree_free(TreeNode *root){
    if (!root) return;
    tree_free(root->left);
    tree_free(root->right);
    free(root);
}

void tree_leaves_to_array(const TreeNode *root, float *out, int *idx_ptr){
    if (!root) return;
    if (root->left || root->right) {
        tree_leaves_to_array(root->left, out, idx_ptr);
        tree_leaves_to_array(root->right, out, idx_ptr);
    }
    else {
        out[*idx_ptr] = root->data;
        (*idx_ptr)++;
    }
}

void tree_leaves_to_array_get_size(const TreeNode *root, int *size_ptr){
    if (!root) return;
    if (root->left || root->right) {
        tree_leaves_to_array_get_size(root->left, size_ptr);
        tree_leaves_to_array_get_size(root->right, size_ptr);
    }
    else {
        (*size_ptr)++;
    }
}
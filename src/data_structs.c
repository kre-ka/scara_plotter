#include "data_structs.h"

#include <stdlib.h>

TreeNode *tree_create_node(float data) {
  TreeNode *node = malloc(sizeof(TreeNode));
  if (!node) exit(EXIT_FAILURE);
  node->data = data;
  node->left = NULL;
  node->right = NULL;
  return node;
}

TreeNode *tree_add_node_left(TreeNode *root, float data) {
  root->left = tree_create_node(data);
  return root->left;
}

TreeNode *tree_add_node_right(TreeNode *root, float data) {
  root->right = tree_create_node(data);
  return root->right;
}

void tree_free(TreeNode *root) {
  if (!root) return;
  tree_free(root->left);
  tree_free(root->right);
  free(root);
}

void tree_leaves_to_array(float **out, const TreeNode *root) {
  int idx = 0;
  _tree_leaves_to_array(out, root, &idx);
}

void _tree_leaves_to_array(float **out, const TreeNode *root, int *idx_ptr) {
  if (!root) return;
  if (root->left || root->right) {
    _tree_leaves_to_array(out, root->left, idx_ptr);
    _tree_leaves_to_array(out, root->right, idx_ptr);
  } else {
    (*out)[*idx_ptr] = root->data;
    (*idx_ptr)++;
  }
}

void tree_leaves_to_array_get_size(int *out_size_ptr, const TreeNode *root) {
  if (!root) return;
  if (root->left || root->right) {
    tree_leaves_to_array_get_size(out_size_ptr, root->left);
    tree_leaves_to_array_get_size(out_size_ptr, root->right);
  } else {
    (*out_size_ptr)++;
  }
}
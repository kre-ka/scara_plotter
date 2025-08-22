#ifndef INC_DATA_STRUCTS_H_
#define INC_DATA_STRUCTS_H_

typedef struct TreeNode TreeNode;
struct TreeNode {
    float data;
    struct TreeNode* left;
    struct TreeNode* right;
};

TreeNode* tree_create_node(float data);
TreeNode* tree_add_node_left(TreeNode *root, float data);
TreeNode* tree_add_node_right(TreeNode *root, float data);
void tree_free(TreeNode *root);
void tree_leaves_to_array(float **out, const TreeNode *root);
void _tree_leaves_to_array(float **out, const TreeNode *root, int *idx_ptr);
void tree_leaves_to_array_get_size(int *out_size_ptr, const TreeNode *root);

#endif /* INC_DATA_STRUCTS_H_ */
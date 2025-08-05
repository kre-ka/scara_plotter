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
void tree_leaves_to_array(const TreeNode *root, float *out, int *idx_ptr);
void tree_leaves_to_array_get_size(const TreeNode *root, int *size_ptr);

#endif /* INC_DATA_STRUCTS_H_ */
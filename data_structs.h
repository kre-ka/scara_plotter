#ifndef INC_DATA_STRUCTS_H_
#define INC_DATA_STRUCTS_H_

struct tree_node {
    float data;
    struct tree_node* left;
    struct tree_node* right;
};
typedef struct tree_node TreeNode;

TreeNode* tree_create_node(float data);
TreeNode* tree_add_node_left(TreeNode *root, float data);
TreeNode* tree_add_node_right(TreeNode *root, float data);
void tree_free(TreeNode *root);
void tree_leaves_to_array(TreeNode *root, float *out, int *idx_ptr);
void tree_leaves_to_array_get_size(TreeNode *root, int *size_ptr);

#endif /* INC_DATA_STRUCTS_H_ */
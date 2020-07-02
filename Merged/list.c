#include "list.h"

struct node* tempNode=NULL;

void init_list(struct list *_list, uint32_t _ID)
{
	// Initialize the list
	_list->root = NULL;
	_list->end=_list->root;
	
	// Set the ID for the list
	_list->ArbID = _ID;
}

void push(struct list *_list, uint8_t *data)
{
	// Check to see if the root node is NULL before doing
	// a recursive push.
	if(_list->root == NULL)
	{
		_list->root = (struct node*)malloc(sizeof(struct node));
		// Copy the data to the root
		memcpy(_list->root->data, data, 8);
		
		_list->end=_list->root;
		_list->root->next = NULL;
		return;
	}
	else
	{
		_list->end->next = (struct node*)malloc(sizeof(struct node));
		// Copy the data to the root
		memcpy(_list->end->next->data, data, 8);
		
		_list->end=_list->end->next;
		_list->end->next = NULL;
		return;
	}
	
	
	// Start the recursive push
	//push_r(_list->root, data);
	
}

void pop(struct list *_list, uint8_t data[8])
{
	// Check to see if the root node is NULL before doing
	// a recursive pop.
	if(_list->root == NULL)
	{
		return;
	}
	// check to see if the root node is the only node in the list
	else
	{
		// Copy the data to the data array
		memcpy(data, _list->root->data, sizeof(uint8_t [8]));
		
		// Set root to NULL to signify the list is empty
		tempNode=_list->root;
		_list->root = _list->root->next;
		free(tempNode);
		
		// return to 
		return;
	}
	// Start the recursive pop
	//pop_r(_list->root, data);
}
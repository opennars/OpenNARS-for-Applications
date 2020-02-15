#include "Stack.h"

void Stack_Push(Stack *stack, VMItem *item)
{
    stack->items[stack->stackpointer] = item;
    stack->stackpointer++;
    assert(stack->stackpointer <= CONCEPTS_MAX, "VMEntry stack overflow");
}

VMItem *Stack_Pop(Stack *stack)
{
    stack->stackpointer--;
    assert(stack->stackpointer >= 0, "VMEntry stack underflow");
    return stack->items[stack->stackpointer];
}

bool Stack_IsEmpty(Stack *stack)
{
    return stack->stackpointer == 0;
}


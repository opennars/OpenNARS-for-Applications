/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define STACK_TEST_STRUCTURE_SIZE 5
void Stack_Test()
{
    puts(">>Stack test start");
    Stack stack = {0};
    VMItem* storageptrs[STACK_TEST_STRUCTURE_SIZE];
    Stack_INIT(&stack, (void**) storageptrs, STACK_TEST_STRUCTURE_SIZE);
    Concept c1 = {0};
    Concept c2 = {0};
    VMItem item1 = { .value = &c1 };
    VMItem item2 = { .value = &c2 };
    Stack_Push(&stack, &item1);
    assert(stack.stackpointer == 1, "Stackpointer wasn't incremented");
    assert(((VMItem**)stack.items)[0]->value == &c1, "Item should point to c1");
    assert(!Stack_IsEmpty(&stack), "Stack should not be empty");
    VMItem *item1_popped = Stack_Pop(&stack);
    assert(stack.stackpointer == 0, "Stackpointer wasn't decremented");
    assert(item1_popped->value == &c1, "Popped item1 should point to c1 (1)");
    Stack_Push(&stack, &item1);
    Stack_Push(&stack, &item2);
    assert(stack.stackpointer == 2, "Stackpointer wrong");
    VMItem *item2_popped = Stack_Pop(&stack);
    assert(item2_popped->value == &c2, "Popped item2 should point to c2");
    VMItem *item1_popped_again = Stack_Pop(&stack);
    assert(item1_popped_again->value == &c1, "Popped item1 should point to c1 (2)");
    assert(Stack_IsEmpty(&stack), "Stack should be empty");
    puts(">>Stack test successul");
}

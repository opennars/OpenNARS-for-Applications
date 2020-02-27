void InvertedAtomIndex_Test()
{
    puts(">>Inverted atom index test start");
    NAR_INIT();
    Term term = Narsese_Term("<a --> (b & c)>");
    Concept c = { .term = term };
    Memory_AddToInvertedAtomIndex(term, &c);
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("a")][0] == &c, "There was no concept reference added for key a!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("b")][0] == &c, "There was no concept reference added for key b!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("c")][0] == &c, "There was no concept reference added for key c!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex(":")][0] == NULL, "There was a concept reference added for key inheritance!");
    Memory_RemoveFromInvertedAtomIndex(term, &c);
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("a")][0] == NULL, "Concept reference was not removed for key a!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("b")][0] == NULL, "Concept reference was not removed for key b!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("c")][0] == NULL, "Concept reference was not removed for key c!");
    Memory_AddToInvertedAtomIndex(term, &c);
    Term term2 = Narsese_Term("<b --> d>");
    Concept c2 = { .term = term2 };
    Memory_AddToInvertedAtomIndex(term2, &c2);
    PrintInvertedAtomIndex();
    Memory_RemoveFromInvertedAtomIndex(term, &c);
    puts("after removal");
    PrintInvertedAtomIndex();
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("a")][0] == NULL, "Concept reference was not removed for key a!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("c")][0] == NULL, "Concept reference was not removed for key c!");
    Memory_RemoveFromInvertedAtomIndex(term, &c);
    puts("after removal");
    PrintInvertedAtomIndex();
    puts(">>Inverted atom index test successul");
}


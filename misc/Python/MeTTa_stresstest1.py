from MeTTa import *

print("""
!(bind! &MeTTaBeliefs (new-space))
(= (MeTTa-OUT (@ ($MeTTaTerm $newTruth) eternal))
   (case (match &MeTTaBeliefs ($MeTTaTerm $existingTruth) ($MeTTaTerm $existingTruth))
         ((($MeTTaTerm $existingTruth)
           (superpose ((remove-atom &MeTTaBeliefs ($MeTTaTerm $existingTruth))
                       (add-atom &MeTTaBeliefs ($MeTTaTerm $newTruth)))))
          (%void% (add-atom &MeTTaBeliefs ($MeTTaTerm $newTruth))))))
""")

NAR_PrintInMetta()
for step in range(100):
    if step % 10 == 0: # a periodic mission knowledge reminder
        NAR_AddInput('!(AddBeliefEvent ((((trash --> (IntSet see)) &/ (^ pick)) =/> G) (1.0 0.9)))')
        NAR_AddInput('!(AddBeliefEvent ((((tick --> (IntSet observed)) &/ (^ go)) =/> (trash --> (IntSet see))) (1.0 0.9)))')
    bottleSeen = random.random() > 0.5
    if bottleSeen:
        print(">>>>BOTTLE SEEN<<<<")
        NAR_AddInput('!(AddBeliefEvent ((bottle --> trash) (1.0 0.9)))')
        NAR_AddInput('!(AddBeliefEvent ((bottle --> (IntSet see)) (1.0 0.9)))')
    else:
        print(">>>>NOTHING SEEN<<<<")
        NAR_AddInput('!(AddBeliefEvent ((tick --> (IntSet observed)) (1.0 0.9)))')
        NAR_AddInput('!(AddBeliefEvent ((trash --> (IntSet see)) (0.0 0.9)))')
    NAR_AddInput("!(EventQuestion (trash --> (IntSet see)))") #does not affect derivations, for monitoring / real-time Q&A!
    NAR_AddInput('!(AddGoalEvent (G (1.0 0.99)))')
    NAR_Cycle(10)

print("!(match &MeTTaBeliefs ((bottle --> (IntSet see)) $1) $1)")

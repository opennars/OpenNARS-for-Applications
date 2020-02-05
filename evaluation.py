from subprocess import PIPE, run
import subprocess
import glob

#YAN C tests & metrics, only print fully output on failure, always print the metrics:
def ctests():
    result = run(['./YAN'], stdout=PIPE, stderr=PIPE, universal_newlines=True)
    if result.returncode != 0:
        print(result.stdout, result.stderr)
        exit(result.returncode)
    for line in result.stdout.split("\n"):
        if "goods=" in line:
            print(line)
    print("\nSystem tests successful!")
ctests()

#Q&A metrics for the Narsese and English files:
TimeCntGlobal = 0
TimeSumGlobal = 0
ConfidenceCntGlobal = 0
ConfidenceSumGlobal = 0.0
def Test(Example, outputString):
    global Timing, ConfidenceCntGlobal, ConfidenceSumGlobal, TimeSumGlobal, TimeCntGlobal
    TimeCnt = 0.0
    TimeSum = 0.0
    ConfidenceCnt = 0.0
    ConfidenceSum = 0.0
    expect_condition = "Comment: expected: "
    expect_condition_answer = expect_condition + "Answer: "
    expect_condition_execution = expect_condition + "^"
    lines = outputString.split('\n')
    for i in range(len(lines)):
        line = lines[i].strip()
        if line.startswith(expect_condition):
            isAnswerCondition = line.startswith(expect_condition_answer)
            isExecutionCondition = line.startswith(expect_condition_execution)
            if isAnswerCondition:
                TruthValue = line.split("Truth:")[1]
                ConfidenceExpected = float(TruthValue.split("confidence=")[1])
                ExpectedOutput = line.split(expect_condition_answer)[1].split("Truth:")[0]
                FoundOutput = False
                ConfidenceObtainedMax = 0
                CreationTimeOfMax = 0
                for j in range(i):
                    line_before = lines[j].strip()
                    if line_before.startswith("Answer:") and ExpectedOutput in line_before:
                        ConfidenceObtained = float(line_before.split("confidence=")[1])
                        CreationTime = int(line_before.split("creationTime=")[1].split(" ")[0])
                        if ConfidenceObtained > ConfidenceObtainedMax:
                            ConfidenceObtainedMax = ConfidenceObtained
                            CreationTimeOfMax = CreationTime
                        FoundOutput = True
                if not FoundOutput or ConfidenceObtainedMax < ConfidenceExpected:
                    print("Failure for " + line + " in " + Example)
                    exit(0)
                TimeSum += float(CreationTime)
                TimeCnt += 1.0
                ConfidenceSum += ConfidenceObtainedMax
                ConfidenceCnt += 1.0
            if isExecutionCondition:
                Message = line.split(expect_condition)[1]
                for j in reversed(range(i)):
                    line_before = lines[j].strip()
                    if line_before.startswith("^"):
                        if Message != line_before:
                            print("Failure for " + line + " in "+ Example)
                            exit(0)
                        else:
                            break
    if TimeCnt > 0:
        print("\nQ&A metrics for test " + Example)
        print("Average answer time = " + str(TimeSum/TimeCnt))
        print("Average answer confidence = " + str(ConfidenceSum/ConfidenceCnt))
        print("Combined loss = " + str((1.0 - ConfidenceSum/ConfidenceCnt) * (TimeSum/TimeCnt)))
        TimeSumGlobal += TimeSum
        TimeCntGlobal += TimeCnt
        ConfidenceSumGlobal += ConfidenceSum
        ConfidenceCntGlobal += ConfidenceCnt

#Evaluate tests & performance on all Narsese examples:
for filename in glob.glob("./examples/nal/*.nal"):
    Test(filename, subprocess.getoutput("cat " + filename + " | ./YAN shell"))
print("\nNarsese integration tests successful!")

#Evaluate tests & performance English examples:
for filename in glob.glob('./examples/english/*.english'):
    Test(filename, subprocess.getoutput("cat " + filename + " | python2 english_shell.py"))
print("\nEnglish integration tests successful!")

#Print global metrics:
if TimeCntGlobal > 0:
    print("\nQ&A metrics global")
    print("Average answer time = " + str(TimeSumGlobal/TimeCntGlobal))
    print("Average answer confidence = " + str(ConfidenceSumGlobal/ConfidenceCntGlobal))
    print("Combined loss = " + str((1.0 - ConfidenceSumGlobal/ConfidenceCntGlobal) * (TimeSumGlobal/TimeCntGlobal)))

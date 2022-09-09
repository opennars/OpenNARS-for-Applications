import os
import glob

lines = []
def AddOutput(line):
    lines.append(line)

print("<html>")
print("<head></head><body>")
print("<center><b>ONA code documentation</b></center><br/><center>Copyright 2022 The OpenNARS authors.</center><hr/>")

modules = []
def Module(name):
    global modules
    modules.append(name)
    AddOutput("<hr/><h1><b>Module: " + name + "</b></h1><br/>")

for filename in glob.glob('./src/*.h'):
    with open(filename) as f:
        text = f.read()
        for line in text.split("\n")[24:]:
            if "#define H_" in line:
                Module(line.split("#define H_")[1]) #.split("#ifndef H_")[0]
            else:
                if (not line.startswith("#") or line.startswith("#define")) and not "//References//" in line and any([x != "/" and x != "-" for x in line]): 
                    isComment = line.startswith("//")
                    if isComment:
                        AddOutput("<br/> <font color=\"green\">")
                    if line.startswith("typedef"):
                        AddOutput("<br/>")
                    AddOutput(line)
                    if isComment:
                        AddOutput("</font>")
                    AddOutput("<br/>")
                
module_output = ["<b>Modules:</b><br/>"]
for i,name in enumerate(modules):
    module_output.append(str(i) + ": " + name + "<br/>")
lines = module_output + lines
AddOutput("</body></html>")
for x in lines:
    print(x)

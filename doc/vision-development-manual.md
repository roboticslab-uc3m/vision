## Building Language Model

### How to add new words for English speech recognition:
You can use this [Sphinx Knowledge Base Tool -- VERSION 3](http://www.speech.cs.cmu.edu/tools/lmtool-new.html).

### How to add new words for Spanish speech recognition:
This will be a quick explanation of how to add new words for Spanish speech recognition.
First of all, you have to know that you need two files. These are the language model (file.lm) and the lexical model (file.dic) referred to variously as the 'pronouncing dictionary' or simply the ‘dictionary’.
For more information, you can check this [link](http://cmusphinx.sourceforge.net/wiki/tutoriallm) about how to build a new Language Model.
If you want to know information about LMs and DIC files and other things, you can check this [link](http://www.speech.cs.cmu.edu/sphinx/doc/sphinx-FAQ.html).

In order to build the dictionary file (dic) you can add the words manually that you need, using this [file]( vision/share/speechRecognition/conf/dictionary/es-20k.dic ) as reference.
You only have to copy the words with the respective secuence of phones to the file .dic that we want to extend.
A word example: **convocar k o n b o k a r **

In the case of language model, you have to use the SRI Language Modeling Toolkit (SRILM). You can download it [here](http://www.speech.sri.com/projects/srilm/download.html). 
* Step 1: Download all the required tools listed at the SRILM download page. 
* Step 2: Download the latest version of SRILM by filling out the form available at the SRILM download page.
* Step 3: Unpack the file inside the folder you desire for SRILM to be installed. 
* Step 4: Open the Makefile inside the folder where SRILM was unpacked using any text editor.
* Step 5: You should find a commented line which looks something like this:
```
# SRILM = /home/speech/stolcke/project/srilm/devel
```
* Remove the "#" character from the beginning of this line and substitute /home/speech/stolcke/project/srilm/devel with the path to the folder where you have unpacked SRILM initially. Example:
```
SRILM = /home/tools/user/srilm
```
* Step 6: Using either a Linux/Mac terminal or a Cygwin terminal, navigate to the folder where you have unpacked SRILM.
* Step 7: Run the command make.

If you have followed this steps correctly, the compiled binaries of SRILM should be found inside the following folder:
```
/home/tools/user/srilm/bin/[your_machine_type]/
```
Prepare a reference text that will be used to generate the language model. In our case, we can use for example "follow-me-spanish.txt" 
To create the language model file, you can do this:
```bash
cd /home/tools/user/srilm/bin/[your_machine_type]/
./ngram-count -wbdiscount -text follow-me-spanish.txt -lm follow-me-spanish.lm
```
Now, you will have the LM file. Remember to copy and replace the old dictionary files located in
 ```/vision/share/speechRecognition/conf/dictionary/``` by the newers.

## Setup Virtual Environtment

1. Open this wit folder
2. Create the virtual environtment "python -m venv ." (.) Means you create a virtual environtment on the opened folder
3. then open command prompt outside the vs code and type ".\Scripts\activate"
4. Install the requirements file "pip install -r requirements.txt" 
5. change mqtt broker address(check your ip with "ipconfig" in cmd)
6. Open src folder "cd src" and Run "python distanceV2.py"

## Run
1. Go to terminal and run 
2. run `cd C:\DaffeydWilbert\Programing\Python\Documented_Project\distanceEstimate`
3. run `.\Scripts\activate`
4. run `cd src`
5. run `python distanceV2.py`
## Tips
1. To automatically generate requirements.txt "python -m  pipreqs.pipreqs [path/to/project]"
2. To freeze library on virtual environtment "pip freeze > requirements.txt"
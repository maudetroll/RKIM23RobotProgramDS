from IPython.display import HTML, display

class HelperClass():

    def __init__(self):
        pass
    
    
    def printInColor(text, farbe='black'):
        formatted_text = "<font color='{}'>{}</font>".format(farbe, text)
        display(HTML(formatted_text))
        

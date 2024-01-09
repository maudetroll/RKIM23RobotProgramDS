from IPython.display import HTML, display

class HelperClass():

    def __init__(self):
        pass
    
    
    def printInColor(text, farbe='black'):
        formatted_text = "<font color='{}'>{}</font>".format(farbe, text)
        display(HTML(formatted_text))
        
    def find_duplicates(solution):
        seen = set()
        duplicates = set()

        for item in solution:
            if item in seen:
                duplicates.add(item)

            else:
                seen.add(item)

        if ('start' in duplicates):
            print("Started planning again from beginning, path will be shorten")

        return list(duplicates)
        

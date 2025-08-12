from flask import Flask
from backend.route_checkdata import bp as checkdata_bp
from backend.route_collect_data import main_bp 

app = Flask(__name__)
app.register_blueprint(checkdata_bp, url_prefix='/check_data')
app.register_blueprint(main_bp)

if __name__ == '__main__':
    app.run(debug=False)
1
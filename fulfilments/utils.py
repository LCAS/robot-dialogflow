from urllib import quote
from requests import get


class Wikipedia:
    # URL_PATTERN='https://simple.wikipedia.org/w/api.php?action=query&format=json&titles=%s&prop=extracts&exintro&explaintext'
    URL_PATTERN = (
        'https://simple.wikipedia.org/w/api.php'
        '?action=opensearch&format=json&search=%s&limit=5&redirects=resolve'
        )

    def query(self, q):
        url = Wikipedia.URL_PATTERN % quote(q)
        r = get(url)
        data = r.json()
        try:
            result = ' '.join(data[2])
            if len(result) > 3:
                return result
            else:
                return "Sorry, I don't know anything about %s." % q
        except:
            return "Sorry, I don't know anything about %s." % q
        # try:
        #     pages = data['query']['pages']
        #     for k in pages:
        #         try:
        #             return pages[k]['extract']
        #         except:
        #             pass
        #     return "I don't know anything about %s." % q
        # except:
        #     return "I don't know anything about %s." % q

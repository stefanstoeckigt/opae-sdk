```mermaid
sequenceDiagram
    participant ClientApp
    ClientApp->>opae: fpgaEnumerate(filter)
    loop ForEach(A in AdapterTables(filter))
        opae->>A: fpgaEnumerate(filter)
        A-->>opae: A_tokens
        loop ForEach(A_tok in A_tokens)
            opae-->>opae: wrap(A_token)
        end
    opae-->>ClientApp:tokens
    end
```